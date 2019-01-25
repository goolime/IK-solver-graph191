#version 330

varying vec2 texCoord0;
varying vec3 normal0;
varying vec3 color0;

uniform vec3 lightDirection;
uniform vec3 lightColor;
uniform vec3 opt;
uniform sampler2D texture1;
//uniform sampler2D texture2;

//out vec4 outputF;

void main()
{
	if (opt.z > 0.0){
		gl_FragColor = vec4(1,1,1/(opt.z), 1.0);
	}
	else{
		vec3 tmp =  dot(-lightDirection, normal0) * (opt.x * texture2D(texture1, texCoord0).xyz+opt.y*color0);
		gl_FragColor = clamp(vec4(tmp, 1.0), 0.0, 1.0);
	}
}
