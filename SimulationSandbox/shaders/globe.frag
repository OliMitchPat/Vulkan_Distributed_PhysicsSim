#version 450

layout(push_constant) uniform PushConstants {
    mat4 model;
    vec4 params; // x=shadingModel, y=shininess, z=dayFactor (0 night..1 day)
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


// Globe pipeline set=1 has TWO textures now:
layout(set = 1, binding = 0) uniform sampler2D skyDay;
layout(set = 1, binding = 1) uniform sampler2D skyNight;

layout(location = 0) in vec3 vtxColor;
layout(location = 1) in vec2 vtxTexCoord;
layout(location = 2) in vec3 vWorldPos;
layout(location = 3) in vec3 vWorldNormal;
layout(location = 4) in vec3 vGouraudLit;

layout(location = 0) out vec4 outColor;

float extractUniformRadius(mat4 M)
{
    float sx = length(vec3(M[0]));
    float sy = length(vec3(M[1]));
    float sz = length(vec3(M[2]));
    return max(sx, max(sy, sz));
}

void main()
{
    float dayFactor = clamp(pushData.params.z, 0.0, 1.0);

    // We treat this pipeline as "globe only".
    // (You can still keep shininess < 0 sentinel on CPU, but the shader doesn't need it.)
    vec3 globeCenter = vec3(pushData.model[3]);
    float distToCamera = length(ubo.cameraPos.xyz - globeCenter);

    float globeRadius = extractUniformRadius(pushData.model);
    bool insideGlobe = distToCamera < globeRadius;

    // Sample both textures once
    vec3 daySky   = texture(skyDay,   vtxTexCoord).rgb;
    vec3 nightSky = texture(skyNight, vtxTexCoord).rgb;

    // Night grading (tweak these)
    const float nightExposure = 0.1;              // lower = darker
    const vec3  nightTint     = vec3(0.04, 0.07, 0.14); // blue-ish tint

    nightSky = nightSky * nightExposure;           // darken
    nightSky = nightSky + nightTint;               // lift blacks with blue
    nightSky = clamp(nightSky, 0.0, 1.0);


    if (insideGlobe)
    {
        // Blend day/night sky textures
        vec3 sky = mix(nightSky, daySky, dayFactor);

        // Sun/Moon discs painted into the sky (match your lighting convention)
        vec3 skyDir = normalize(vWorldPos - globeCenter);

        vec3 sunDir  = normalize(-ubo.sunDir_Int.xyz);
        vec3 moonDir = normalize(-ubo.moonDir_Int.xyz);

        float sunDot  = dot(skyDir, sunDir);
        float moonDot = dot(skyDir, moonDir);

        // Disc sizes (tweak if needed)
        float sunDisc  = smoothstep(0.9994,  0.9999,  sunDot);
        float moonDisc = smoothstep(0.9996,  0.99995, moonDot);

        vec3 sunCol  = vec3(1.0, 0.95, 0.80) * max(ubo.sunDir_Int.w, 0.0);
        vec3 moonCol = vec3(0.60, 0.70, 1.00) * max(ubo.moonDir_Int.w, 0.0);

        float sunVis  = dayFactor;
        float moonVis = 1.0 - dayFactor;

        // Boost a bit for visibility
        sky += sunDisc  * sunCol  * sunVis  * 10.0;
        sky += moonDisc * moonCol * moonVis * 2.5;

        outColor = vec4(sky, 1.0);
        return;
    }
    else
    {
        // Outside: glass shell (no texture)
        vec3 glassTint = vec3(0.8, 0.85, 0.9);
        float alpha = 0.12;

        float n2 = dot(vWorldNormal, vWorldNormal);
        vec3 N = (n2 > 1e-8) ? normalize(vWorldNormal) : vec3(0.0, 1.0, 0.0);
        vec3 V = normalize(ubo.cameraPos.xyz - vWorldPos);

        // Fresnel-ish rim
        float rim = pow(1.0 - max(dot(N, V), 0.0), 3.0);
        vec3 color = glassTint + rim * 0.25;

        outColor = vec4(color, alpha);
        return;
    }
}
