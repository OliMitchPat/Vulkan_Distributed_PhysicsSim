#version 450

layout(push_constant) uniform PushConstants {
    mat4 model;
    vec4 params; // x=shadingModel, y=shininess
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

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec2 inTexCoord;
layout(location = 3) in vec3 inNormal;

layout(location = 0) out vec3 vtxColor;
layout(location = 1) out vec2 vtxTexCoord;
layout(location = 2) out vec3 vWorldPos;
layout(location = 3) out vec3 vWorldNormal;
layout(location = 4) out vec3 vGouraudLit; // per-vertex lighting result

vec3 evalDirectionalLight(vec3 N, vec3 V, vec3 lightDir, float intensity, float shininess)
{
    // Convention: lightDir = direction light travels (sun -> scene)
    vec3 L = normalize(-lightDir); // from surface -> light
    float NdotL = max(dot(N, L), 0.0);

    // Blinn-Phong (stable, good for coursework)
    vec3 H = normalize(L + V);
    float spec = (NdotL > 0.0) ? pow(max(dot(N, H), 0.0), shininess) : 0.0;

    vec3 diffuse  = vec3(NdotL) * intensity;
    vec3 specular = vec3(spec)  * intensity;

    return diffuse + specular;
}

void main()
{
    vec4 worldPos4 = pushData.model * vec4(inPosition, 1.0);
    vWorldPos = worldPos4.xyz;

    mat3 normalMat = transpose(inverse(mat3(pushData.model)));
    vec3 n = normalMat * inNormal;

    float n2 = dot(n, n);
    vWorldNormal = (n2 > 1e-8) ? normalize(n) : vec3(0.0, 1.0, 0.0);


    vtxTexCoord = inTexCoord;
    vtxColor = inColor;

    vec3 V = normalize(ubo.cameraPos.xyz - vWorldPos);
    float shininess = pushData.params.y;

    vec3 lightSum = vec3(0.0);
    lightSum += evalDirectionalLight(vWorldNormal, V, ubo.sunDir_Int.xyz,  ubo.sunDir_Int.w,  shininess);
    lightSum += evalDirectionalLight(vWorldNormal, V, ubo.moonDir_Int.xyz, ubo.moonDir_Int.w, shininess);

    // Ambient stays separate
    vGouraudLit = ubo.ambient.rgb + lightSum;

    gl_Position = ubo.proj * ubo.view * worldPos4;
}
