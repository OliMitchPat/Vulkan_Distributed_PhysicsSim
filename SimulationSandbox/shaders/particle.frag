#version 450
layout(location = 0) in vec2 vUV;
layout(location = 1) in vec4 vColor;
layout(location = 0) out vec4 outColor;

void main()
{
    // Convert UV (0..1) into centered coords (-1..1)
    vec2 d = vUV * 2.0 - 1.0;

    // Circle test
    float r2 = dot(d, d);
    if (r2 > 1.0)
        discard;

    // Soft edge so it looks less harsh
    float soft = 1.0 - smoothstep(0.6, 1.0, r2);

    outColor = vec4(vColor.rgb, vColor.a * soft);
}

