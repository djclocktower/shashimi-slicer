#version 110

// VHS playback: smeared chroma that trails to the right, sharpened luma, tape color, scan lines,
// grain, dropouts and tracking wobble with a noisy band that rolls up the screen.

uniform sampler2D uniform_texture;
uniform vec2 inv_tex_size;
// seconds; drives the noise and the tracking band
uniform float time;
// framebuffer pixels per scan line
uniform float line_scale;

varying vec2 tex_coord;

const mat3 RGB_TO_YIQ = mat3(0.299, 0.596, 0.211, 0.587, -0.274, -0.523, 0.114, -0.322, 0.312);
const mat3 YIQ_TO_RGB = mat3(1.0, 1.0, 1.0, 0.956, -0.272, -1.106, 0.621, -0.647, 1.703);

float hash(vec2 p)
{
    return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453);
}

vec3 yiq(vec2 uv)
{
    return RGB_TO_YIQ * texture2D(uniform_texture, uv).rgb;
}

void main()
{
    float row   = floor(gl_FragCoord.y / line_scale);
    float frame = floor(time * 30.0);
    float step_x = inv_tex_size.x * line_scale;

    // Tracking: every line wobbles slightly, much more inside the band rolling up the screen.
    float band_y  = 1.0 - fract(time * 0.04);
    float in_band = 1.0 - smoothstep(0.0, 0.035, abs(tex_coord.y - band_y));
    float wobble  = sin(row * 0.35 + time * 3.0) * 0.4 + (hash(vec2(row, frame)) - 0.5) * (0.6 + 10.0 * in_band);
    vec2 uv = tex_coord + vec2(wobble * step_x, 0.0);

    // Luma keeps its detail, with the edge ringing of the deck's sharpening.
    float y      = yiq(uv).x;
    float y_blur = 0.5 * (yiq(uv - vec2(2.0 * step_x, 0.0)).x + yiq(uv + vec2(2.0 * step_x, 0.0)).x);
    y += (y - y_blur) * 0.5;

    // Chroma has a fraction of the bandwidth: smeared, and trailing to the right.
    vec2 iq = vec2(0.0);
    for (int i = 0; i < 6; ++i)
        iq += yiq(uv - vec2(float(i) * 1.5 * step_x, 0.0)).yz;
    iq /= 6.0;

    // Tape color: lifted blacks, a little warm and over-saturated.
    y = y * 0.88 + 0.07;
    iq *= 1.15;
    iq.x += 0.015;
    vec3 color = YIQ_TO_RGB * vec3(y, iq);

    // Scan lines.
    color *= mod(row, 2.0) < 1.0 ? 1.0 : 0.62;

    // Grain, stronger in the tracking band, and rare white dropout streaks.
    color += (hash(gl_FragCoord.xy + frame) - 0.5) * (0.06 + 0.25 * in_band);
    float dropout = step(0.9985, hash(vec2(row, frame))) * step(0.6, hash(vec2(floor(gl_FragCoord.x / (40.0 * line_scale)), row + frame)));
    color = mix(color, vec3(0.9), dropout * 0.8);

    // The television's vignette.
    vec2 d = tex_coord - 0.5;
    color *= 1.0 - dot(d, d) * 0.45;

    gl_FragColor = vec4(clamp(color, 0.0, 1.0), 1.0);
}
