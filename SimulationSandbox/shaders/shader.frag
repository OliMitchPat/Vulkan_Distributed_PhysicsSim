#version 450

layout(push_constant) uniform PushConstants {
    mat4 model;
    vec4 params;
    vec4 objectColor;
} pushData;

#define MAX_SPARK_LIGHTS 32

struct PointLight {
    vec4 pos_radius; // xyz = position, w = radius
    vec4 col_int;    // rgb = color,    w = intensity
};

layout(set = 0, binding = 0) uniform UniformBufferObject {
    mat4 view;
    mat4 proj;
    vec4 cameraPos;
    vec4 sunDir_Int;
    vec4 moonDir_Int;
    vec4 ambient;

    PointLight sparkLights[MAX_SPARK_LIGHTS];
    ivec4 sparkLightCount_pad; // x = count
} ubo;

layout(set = 1, binding = 0) uniform sampler2D texSampler;

layout(location = 0) in vec3 vtxColor;
layout(location = 1) in vec2 vtxTexCoord;
layout(location = 2) in vec3 vWorldPos;
layout(location = 3) in vec3 vWorldNormal;
layout(location = 4) in vec3 vGouraudLit;

layout(location = 0) out vec4 outColor;

vec3 evalDirectionalLight(vec3 N, vec3 V, vec3 lightDir, float intensity, float shininess)
{
    vec3 L = normalize(-lightDir);
    float NdotL = max(dot(N, L), 0.0);

    vec3 H = normalize(L + V);
    float spec = (NdotL > 0.0) ? pow(max(dot(N, H), 0.0), shininess) : 0.0;

    vec3 diffuse  = vec3(NdotL) * intensity;
    vec3 specular = vec3(spec)  * intensity;

    return diffuse + specular;
}

// NEW: short-range point light (spark) with smooth falloff
vec3 evalPointLight(vec3 worldPos, vec3 N,
                    vec3 lightPos, vec3 lightColor,
                    float intensity, float radius)
{
    vec3 toL = lightPos - worldPos;
    float dist = length(toL);

    if (dist > radius) return vec3(0.0);

    vec3 L = toL / max(dist, 1e-4);
    float NdotL = max(dot(N, L), 0.0);

    // Smooth falloff to 0 at radius
    float att = 1.0 - (dist / radius);
    att = att * att; // quadratic

    return lightColor * (intensity * att * NdotL);
}

float extractUniformRadius(mat4 M)
{
    float sx = length(vec3(M[0]));
    float sy = length(vec3(M[1]));
    float sz = length(vec3(M[2]));
    return max(sx, max(sy, sz));
}

void main()
{
    vec4 tex = texture(texSampler, vtxTexCoord);
    vec3 albedo = tex.rgb * vtxColor;

    float shadingModel = pushData.params.x;
    float shininess    = pushData.params.y;
    float dayFactor    = clamp(pushData.params.z, 0.0, 1.0);

    // -----------------------------
    // Emissive spheres (sun/moon outside)
    // -----------------------------
    bool isEmissiveSun  = (shininess < -2.5 && shininess > -3.5); // -3
    bool isEmissiveMoon = (shininess < -3.5 && shininess > -4.5); // -4

    if (isEmissiveSun)
    {
        outColor = vec4(tex.rgb * 8.0, 1.0); // sun brighter
        return;
    }
    if (isEmissiveMoon)
    {
        outColor = vec4(tex.rgb * 1.5, 1.0); // moon dimmer
        return;
    }

    // -----------------------------
    // Globe (sky inside, glass outside)
    // -----------------------------
    bool isGlobe = (shininess < 0.0);

    if (isGlobe)
    {
        vec3 globeCenter = vec3(pushData.model[3]);
        float distToCamera = length(ubo.cameraPos.xyz - globeCenter);

        float globeRadius = extractUniformRadius(pushData.model);
        bool insideGlobe = distToCamera < globeRadius;

        if (insideGlobe)
        {
            vec3 daySky   = tex.rgb;
            vec3 nightSky = tex.rgb * vec3(0.04, 0.06, 0.10);
            vec3 sky = mix(nightSky, daySky, dayFactor);

            vec3 skyDir = normalize(vWorldPos - globeCenter);

            vec3 sunDir  = normalize(-ubo.sunDir_Int.xyz);
            vec3 moonDir = normalize(-ubo.moonDir_Int.xyz);

            float sunDot  = dot(skyDir, sunDir);
            float moonDot = dot(skyDir, moonDir);

            float sunDisc  = smoothstep(0.9994, 0.9999, sunDot);
            float moonDisc = smoothstep(0.9996, 0.99995, moonDot);

            vec3 sunCol  = vec3(1.0, 0.95, 0.80) * max(ubo.sunDir_Int.w, 0.0);
            vec3 moonCol = vec3(0.60, 0.70, 1.00) * max(ubo.moonDir_Int.w, 0.0);

            float sunVis  = dayFactor;
            float moonVis = 1.0 - dayFactor;

            sky += sunDisc  * sunCol  * sunVis * 10.0;
            sky += moonDisc * moonCol * moonVis * 2.5;

            outColor = vec4(sky, 1.0);
            return;
        }
        else
        {
            vec3 glassTint = vec3(0.8, 0.85, 0.9);
            float alpha = 0.12;

            float n2 = dot(vWorldNormal, vWorldNormal);
            vec3 N = (n2 > 1e-8) ? normalize(vWorldNormal) : vec3(0.0, 1.0, 0.0);
            vec3 V = normalize(ubo.cameraPos.xyz - vWorldPos);

            float rim = pow(1.0 - max(dot(N, V), 0.0), 3.0);
            vec3 color = glassTint + rim * 0.25;

            outColor = vec4(color, alpha);
            return;
        }
    }

    // -----------------------------
    // NORMAL OBJECT PATH (+ SPARK LIGHTS)
    // -----------------------------
    vec3 lit;
    if (shadingModel < 0.5) {
        lit = vGouraudLit;
    } else {
        float n2 = dot(vWorldNormal, vWorldNormal);
        vec3 N = (n2 > 1e-8) ? normalize(vWorldNormal) : vec3(0.0, 1.0, 0.0);

        vec3 V = normalize(ubo.cameraPos.xyz - vWorldPos);

        vec3 lightSum = vec3(0.0);
        lightSum += evalDirectionalLight(N, V, ubo.sunDir_Int.xyz,  ubo.sunDir_Int.w,  shininess);
        lightSum += evalDirectionalLight(N, V, ubo.moonDir_Int.xyz, ubo.moonDir_Int.w, shininess);

        // NEW: add spark point lights
        int count = clamp(ubo.sparkLightCount_pad.x, 0, MAX_SPARK_LIGHTS);
        for (int i = 0; i < count; ++i)
        {
            vec3 lp = ubo.sparkLights[i].pos_radius.xyz;
            float radius = ubo.sparkLights[i].pos_radius.w;

            vec3 col = ubo.sparkLights[i].col_int.rgb;
            float intensity = ubo.sparkLights[i].col_int.w;

            lightSum += evalPointLight(vWorldPos, N, lp, col, intensity, radius);
        }

        lit = ubo.ambient.rgb + lightSum;
    }

    outColor = vec4(vtxColor, pushData.objectColor.a);
}
