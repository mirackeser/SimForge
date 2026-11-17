#pragma once

namespace simforge::shaders {

inline constexpr const char* surfaceVertex = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uModel, uView, uProjection, uLightMatrix;
out vec3 vNormal;
out vec3 vWorldPos;
out vec4 vShadowPos;
void main() {
  vec4 world = uModel * vec4(aPos, 1.0);
  vWorldPos = world.xyz;
  vNormal = mat3(transpose(inverse(uModel))) * aNormal;
  vShadowPos = uLightMatrix * world;
  gl_Position = uProjection * uView * world;
}
)";

inline constexpr const char* surfaceFragment = R"(#version 330 core
in vec3 vNormal;
in vec3 vWorldPos;
in vec4 vShadowPos;
uniform vec3 uColor, uCameraPos;
uniform float uMetallic, uRoughness;
uniform int uSurface;
uniform sampler2D uShadowMap;
out vec4 FragColor;
const float PI = 3.14159265;

float hash(vec2 p) {
  return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}
float noise(vec2 p) {
  vec2 i = floor(p), f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x),
             mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}
float stoneSeam(vec2 p) {
  vec2 cell = floor(p), f = fract(p);
  float first = 10.0, second = 10.0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y) {
      vec2 offset = vec2(x, y);
      vec2 seed = vec2(hash(cell + offset), hash(cell + offset + 13.7));
      float distance = length(offset + seed - f);
      if (distance < first) {
        second = first;
        first = distance;
      } else
        second = min(second, distance);
    }
  return 1.0 - smoothstep(0.008, 0.035, second - first);
}
float visibility(vec3 n, vec3 l) {
  vec3 p = vShadowPos.xyz / vShadowPos.w * 0.5 + 0.5;
  if (p.z > 1.0 || p.z < 0.0) return 1.0;
  float bias = max(0.0006 * (1.0 - dot(n, l)), 0.00012);
  vec2 pixel = 1.0 / vec2(textureSize(uShadowMap, 0));
  float result = 0.0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y) {
      result += p.z - bias <= texture(uShadowMap, p.xy + vec2(x, y) * pixel).r
                    ? 1.0
                    : 0.0;
    }
  return result / 9.0;
}
vec3 environment(vec3 direction, float roughness) {
  vec3 base = mix(vec3(0.025, 0.035, 0.05), vec3(0.23, 0.29, 0.36),
                  smoothstep(-0.15, 0.8, direction.y));
  float strip = pow(max(0.0, dot(direction, normalize(vec3(-0.5, 0.7, 0.3)))),
                    mix(100.0, 4.0, roughness));
  float rim = pow(max(0.0, dot(direction, normalize(vec3(0.7, 0.3, -0.6)))),
                  mix(160.0, 5.0, roughness));
  return base + strip * vec3(3.8, 3.4, 2.8) + rim * vec3(0.6, 1.2, 1.8);
}
vec3 lightBRDF(vec3 n, vec3 v, vec3 l, vec3 radiance, vec3 albedo,
               float metallic, float roughness) {
  vec3 h = normalize(v + l);
  float nv = max(dot(n, v), 0.001), nl = max(dot(n, l), 0.0),
        nh = max(dot(n, h), 0.0);
  float alpha = roughness * roughness, alpha2 = alpha * alpha;
  float denominator = nh * nh * (alpha2 - 1.0) + 1.0;
  float distribution = alpha2 / max(PI * denominator * denominator, 0.00001);
  float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
  float geometry = nv / (nv * (1.0 - k) + k) * nl / (nl * (1.0 - k) + k);
  vec3 f0 = mix(vec3(0.04), albedo, metallic);
  vec3 fresnel = f0 + (1.0 - f0) * pow(1.0 - max(dot(h, v), 0.0), 5.0);
  vec3 specular =
      distribution * geometry * fresnel / max(4.0 * nv * nl, 0.0001);
  return ((1.0 - fresnel) * (1.0 - metallic) * albedo / PI + specular) *
         radiance * nl;
}
void main() {
  vec3 n = normalize(vNormal), albedo = pow(uColor, vec3(2.2));
  float roughness = clamp(uRoughness, 0.08, 1.0),
        metallic = clamp(uMetallic, 0.0, 1.0);
  if (uSurface == 1) {
    vec2 p = vWorldPos.xz;
    float grain =
        noise(p * 4.0) * 0.55 + noise(p * 31.0) * 0.25 + noise(p * 120.0) * 0.2;
    float seam = stoneSeam(p * 0.55 + noise(p * 0.7) * 0.15);
    albedo *= (0.65 + grain * 0.8) * (1.0 - seam * 0.58);
    n = normalize(
        n + vec3((noise(p * 13.0 + 0.03) - noise(p * 13.0 - 0.03)) * 0.14, 0.0,
                 (noise(p * 17.0 + 0.02) - noise(p * 17.0 - 0.02)) * 0.14));
  } else if (uSurface == 2) {
    float grain = noise(vWorldPos.xz * vec2(2.0, 70.0));
    albedo *= 0.75 + 0.35 * grain;
  }
  vec3 v = normalize(uCameraPos - vWorldPos),
       l = normalize(vec3(-0.5, 0.85, 0.4));
  vec3 color =
      lightBRDF(n, v, l, vec3(5.0, 4.6, 4.0), albedo, metallic, roughness) *
      visibility(n, l);
  color += lightBRDF(n, v, normalize(vec3(0.7, 0.4, -0.6)),
                     vec3(0.45, 0.8, 1.25), albedo, metallic, roughness);
  vec3 f0 = mix(vec3(0.04), albedo, metallic);
  vec3 fresnel = f0 + (1.0 - f0) * pow(1.0 - max(dot(n, v), 0.0), 5.0);
  color += albedo * (1.0 - metallic) * vec3(0.09, 0.12, 0.16);
  color += environment(reflect(-v, n), roughness) * fresnel *
           (1.0 - 0.55 * roughness);
  float fog = 1.0 - exp(-length(uCameraPos - vWorldPos) * 0.012);
  color = mix(color, vec3(0.025, 0.035, 0.05), fog);
  color = clamp(
      (color * (2.51 * color + 0.03)) / (color * (2.43 * color + 0.59) + 0.14),
      0.0, 1.0);
  FragColor = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);
}
)";

inline constexpr const char* depthVertex = R"(#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uModel, uLightMatrix;
void main() { gl_Position = uLightMatrix * uModel * vec4(aPos, 1.0); }
)";
inline constexpr const char* depthFragment = R"(#version 330 core
void main() {}
)";

}  // namespace simforge::shaders
