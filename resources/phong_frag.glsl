#version 120

varying vec3 vPos; // in camera space
varying vec3 vNor; // in camera space
varying vec2 vTex;
uniform vec3 kdFront;
uniform vec3 kdBack;
uniform sampler2D kdTex;

void main()
{
	vec3 lightPos = vec3(0.0, 0.0, 0.0);
	vec3 n = normalize(vNor);
	vec3 l = normalize(lightPos - vPos);
	vec3 v = -normalize(vPos);
	vec3 h = normalize(l + v);
	// vec3 kd = kdFront;
	float ln = max(dot(l, n), 0.0);
	// if(ln < 0.0) {
	// 	kd = kdBack;
	// 	ln = -ln;
	// }
	vec4 colorT = texture2D(kdTex, vTex.st).rgba;
	vec3 diffuse = ln * colorT.rgb;
	// vec3 colorS = pow(max(dot(h, n), 0.0), s) * vec3(0.2, 0.5, 0.2);
	vec3 color = diffuse;
	gl_FragColor = vec4(color, 0.5);
}
