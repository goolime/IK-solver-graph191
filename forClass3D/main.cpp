#include <Windows.h>
#include <iostream>
#include "display.h"
#include "mesh.h"
#include "shader.h"
#include "inputManager.h"
#include "texture.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include "main.h"
using namespace std;
using namespace glm;

static const int DISPLAY_WIDTH = 1000;
static const int DISPLAY_HEIGHT = 1000;
static const float min_angle = 0.1f;
static const float global_scale = 0.15f;
static const float robot_scale = 2;
static const int robot_parts = 4;
static mat4 positions[robot_parts];
static mat4 rotation[robot_parts];
static int index = -1;
static mat4 M;
static int solve = -1;
static int pick = -1;
static mat4 arm_position = mat4(1.f);
static mat4 qube_position = glm::translate(glm::vec3(5.f, 0.f, 0.f));
static mat4 qube_rotation = mat4(1.f);
static vec3 pc, pt, pe, pepc, ptpc;
int x, y;
float theta, psi, phi;
int euler = -1;




vec4 get_end_point(int rp) {
	mat4 dist = glm::translate((glm::vec3(0, 0, 2 * robot_scale)));
	mat4 tmp;
	vec4 loc = glm::vec4(0.f, 0.f, 0.f, 1.f);
	mat4 MVP_t = rotation[0] * dist;
	for (int j = 1; j <= rp; j++) {
		MVP_t = MVP_t * rotation[j] * dist;
	}
	return arm_position * MVP_t * loc;
}


float calc_dist() {
	vec3 qp = vec3(qube_position * glm::vec4(0.f, 0.f, 0.f, 1.f));
	vec3 ge = vec3(get_end_point(robot_parts-1));
	float ans = glm::distance(qp,ge);
	return ans;
}

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

mat4 get_euler_mat(float phi,float psi, float theta) {
	float r_phi, r_psi, r_theta;
	
	r_phi = (phi / 180) * 3.14159265359;
	r_psi = (psi / 180) * 3.14159265359;
	r_theta = (theta / 180) * 3.14159265359;
	
	return glm::rotate(phi, vec3(0.f, 0.f, 1.f)) * glm::rotate(theta, vec3(1.f, 0.f, 0.f)) * glm::rotate(0.f, vec3(0.f, 0.f, 1.f));
}

void calc_euler(glm::mat4 mat) {
	float sy = sqrt(mat[0][0] * mat[0][0] + mat[1][0] *mat[1][0]);

	bool singular = sy < 1e-6;

	if (!singular)
	{
		psi = atan2(mat[2][1], mat[2][2]);
		theta = atan2(-mat[2][0], sy);
		phi = atan2(mat[1][0], mat[0][0]);
	}
	else
	{
		psi = atan2(-mat[1][2], mat[1][1]);
		theta = atan2(-mat[2][0], sy);
		phi = 0;
	}
	
	psi = (psi / 3.14159265359) * 180;
	theta = (theta / 3.14159265359) * 180;
	phi = (phi / 3.14159265359) * 180;
}

vec3 AAtoEuler(vec3 axis, float angle) {
	float phi, psi, theta;
	float s = glm::sin(angle);
	float c = glm::cos(angle);
	float t = 1 - c;
	
	if ((axis.x*axis.y*t + axis.z * s) > 0.999999999) { // north pole singularity detected
		phi = 2 * atan2(axis.x*glm::sin(angle / 2), glm::cos(angle / 2));
		theta = 3.14159265359 / 2;
		psi = 0;
		//return;
	}
	else if ((axis.x*axis.y*t + axis.z * s) < -0.999999999) { // south pole singularity detected
		phi = -2 * atan2(axis.x*glm::sin(angle / 2), glm::cos(angle / 2));
		theta = -3.14159265359 / 2;
		psi = 0;
		//return;
	}
	else {
	
		phi = atan2(axis.y * s - axis.x * axis.z * t, 1 - (axis.y*axis.y + axis.z * axis.z) * t);
		theta = asin(axis.x * axis.y * t + axis.z * s);
		psi = atan2(axis.x * s - axis.y * axis.z * t, 1 - (axis.x*axis.x + axis.z * axis.z) * t);
	}

	return vec3(phi, theta, psi);
}

int main(int argc, char** argv)
{
	
	Display display(DISPLAY_WIDTH, DISPLAY_HEIGHT, "HW4 | Robot Arm | Gal Meir - 305382137");
	
	Vertex vertices[] =
	{
		Vertex(glm::vec3(-1, -1, -1), glm::vec2(1, 0), glm::vec3(0, 0, -1),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(-1, 1, -1), glm::vec2(0, 0), glm::vec3(0, 0, -1),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(1, 1, -1), glm::vec2(0, 1), glm::vec3(0, 0, -1),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(1, -1, -1), glm::vec2(1, 1), glm::vec3(0, 0, -1),glm::vec3(1, 0, 0)),

		Vertex(glm::vec3(-1, -1, 1), glm::vec2(1, 0), glm::vec3(0, 0, 1),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(-1, 1, 1), glm::vec2(0, 0), glm::vec3(0, 0, 1),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(1, 1, 1), glm::vec2(0, 1), glm::vec3(0, 0, 1),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(1, -1, 1), glm::vec2(1, 1), glm::vec3(0, 0, 1),glm::vec3(1, 0, 0)),

		Vertex(glm::vec3(-1, -1, -1), glm::vec2(0, 1), glm::vec3(0, -1, 0),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(-1, -1, 1), glm::vec2(1, 1), glm::vec3(0, -1, 0),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(1, -1, 1), glm::vec2(1, 0), glm::vec3(0, -1, 0),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(1, -1, -1), glm::vec2(0, 0), glm::vec3(0, -1, 0),glm::vec3(1, 0, 0)),

		Vertex(glm::vec3(-1, 1, -1), glm::vec2(0, 1), glm::vec3(0, 1, 0),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(-1, 1, 1), glm::vec2(1, 1), glm::vec3(0, 1, 0),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(1, 1, 1), glm::vec2(1, 0), glm::vec3(0, 1, 0),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(1, 1, -1), glm::vec2(0, 0), glm::vec3(0, 1, 0),glm::vec3(1, 0, 0)),

		Vertex(glm::vec3(-1, -1, -1), glm::vec2(1, 1), glm::vec3(-1, 0, 0),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(-1, -1, 1), glm::vec2(1, 0), glm::vec3(-1, 0, 0),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(-1, 1, 1), glm::vec2(0, 0), glm::vec3(-1, 0, 0),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(-1, 1, -1), glm::vec2(0, 1), glm::vec3(-1, 0, 0),glm::vec3(1, 0, 0)),

		Vertex(glm::vec3(1, -1, -1), glm::vec2(1, 1), glm::vec3(1, 0, 0),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(1, -1, 1), glm::vec2(1, 0), glm::vec3(1, 0, 0),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(1, 1, 1), glm::vec2(0, 0), glm::vec3(1, 0, 0),glm::vec3(1, 0, 0)),
		Vertex(glm::vec3(1, 1, -1), glm::vec2(0, 1), glm::vec3(1, 0, 0),glm::vec3(1, 0, 0))
	};

	Vertex vertices_axis[] =
	{
		Vertex(glm::vec3(-1, -1, -1), glm::vec2(1, 0), glm::vec3(0, 0, -1),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(-1, 1, -1), glm::vec2(0, 0), glm::vec3(0, 0, -1),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(1, 1, -1), glm::vec2(0, 1), glm::vec3(0, 0, -1),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(1, -1, -1), glm::vec2(1, 1), glm::vec3(0, 0, -1),glm::vec3(0, 0, 0)),

		Vertex(glm::vec3(-1, -1, 1), glm::vec2(1, 0), glm::vec3(0, 0, 1),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(-1, 1, 1), glm::vec2(0, 0), glm::vec3(0, 0, 1),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(1, 1, 1), glm::vec2(0, 1), glm::vec3(0, 0, 1),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(1, -1, 1), glm::vec2(1, 1), glm::vec3(0, 0, 1),glm::vec3(0, 0, 0)),

		Vertex(glm::vec3(-1, -1, -1), glm::vec2(0, 1), glm::vec3(0, -1, 0),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(-1, -1, 1), glm::vec2(1, 1), glm::vec3(0, -1, 0),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(1, -1, 1), glm::vec2(1, 0), glm::vec3(0, -1, 0),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(1, -1, -1), glm::vec2(0, 0), glm::vec3(0, -1, 0),glm::vec3(0, 0, 0)),

		Vertex(glm::vec3(-1, 1, -1), glm::vec2(0, 1), glm::vec3(0, 1, 0),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(-1, 1, 1), glm::vec2(1, 1), glm::vec3(0, 1, 0),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(1, 1, 1), glm::vec2(1, 0), glm::vec3(0, 1, 0),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(1, 1, -1), glm::vec2(0, 0), glm::vec3(0, 1, 0),glm::vec3(0, 0, 0)),

		Vertex(glm::vec3(-1, -1, -1), glm::vec2(1, 1), glm::vec3(-1, 0, 0),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(-1, -1, 1), glm::vec2(1, 0), glm::vec3(-1, 0, 0),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(-1, 1, 1), glm::vec2(0, 0), glm::vec3(-1, 0, 0),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(-1, 1, -1), glm::vec2(0, 1), glm::vec3(-1, 0, 0),glm::vec3(0, 0, 0)),

		Vertex(glm::vec3(1, -1, -1), glm::vec2(1, 1), glm::vec3(1, 0, 0),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(1, -1, 1), glm::vec2(1, 0), glm::vec3(1, 0, 0),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(1, 1, 1), glm::vec2(0, 0), glm::vec3(1, 0, 0),glm::vec3(0, 0, 0)),
		Vertex(glm::vec3(1, 1, -1), glm::vec2(0, 1), glm::vec3(1, 0, 0),glm::vec3(0, 0, 0))
	};

	unsigned int indices[] = {0, 1, 2,
							  0, 2, 3,

							  6, 5, 4,
							  7, 6, 4,

							  10, 9, 8,
							  11, 10, 8,

							  12, 13, 14,
							  12, 14, 15,

							  16, 17, 18,
							  16, 18, 19,

							  22, 21, 20,
							  23, 22, 20
	                          };
    Mesh mesh(vertices, sizeof(vertices)/sizeof(vertices[0]), indices, sizeof(indices)/sizeof(indices[0]));
	Mesh mesh_axis(vertices_axis, sizeof(vertices_axis) / sizeof(vertices_axis[0]), indices, sizeof(indices) / sizeof(indices[0]));
	Shader shader("./res/shaders/basicShader");

	
	positions[0]= glm::translate(glm::vec3(0.f, 0.f, robot_scale));
	for (int i = 1; i < robot_parts; i++) {
		positions[i]= glm::translate(glm::vec3(0.f, 0.f, robot_scale));
	}

	
	for (int i = 0; i < robot_parts; i++) {
		rotation[i] = glm::mat4(1.f);
	}

	mat4 axis_translate = glm::translate(vec3(0.f, 0.f, 0.f));
	mat4 axis_scale = glm::scale(glm::vec3(4.f,0.03f,0.03f));
	
	
	vec3 forward = glm::vec3(0.0f, 0.0f, 1.0f);
	vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
	mat4 P = glm::perspective(60.0f, (float)DISPLAY_WIDTH/(float)DISPLAY_HEIGHT, 0.1f, 100.0f);
	M = glm::rotate(-90.0f,vec3(1,0,0));
	mat4 RS = glm::scale(glm::vec3(1.f,1.f, robot_scale));
	mat4 GS = glm::scale(glm::vec3(global_scale));
	mat4 T = glm::translate(glm::vec3(0.f, 1.f, 0.f));
	static vec3 pos = glm::vec3(0, 0, -5);
	P = P * glm::lookAt(pos, pos + forward, up);
	mat4 MVP = P*M;


	glfwSetKeyCallback(display.m_window,key_callback);
	glfwSetMouseButtonCallback(display.m_window, mouse_callback);
	glfwSetCursorPosCallback(display.m_window, cursor_position_callback);
	glfwSetScrollCallback(display.m_window, scroll_callback);


	mat4 MVP_t = glm::mat4(1.f);

	mat4 axis_rotation[3] = {glm::rotate(90.0f, vec3(0, 0, 1)), glm::rotate(90.0f, vec3(0, 1, 0)), glm::mat4(1.f) };
	
	
	vec3 opt_axis = vec3(0.f, 1.f,0.f);
	vec3 opt_arm = vec3(1.f, 0.f,0.f);
	vec2 opt_pic = vec2(0.f, 0.f);
	
	
	Texture tex1("res/textures/box0.bmp");
	Texture tex2("res/textures/grass.bmp");
	
	while (!glfwWindowShouldClose(display.m_window))
	{
		float dist = calc_dist();
		float t_dist = glm::distance(qube_position*vec4(0, 0, 0, 1.f), arm_position*vec4(0, 0, 0, 1.f));
		vec3 pe = vec3(get_end_point(robot_parts - 1));
		
		if (solve == 1 && t_dist > 2 * robot_scale*robot_parts) {
			solve = solve * -1;
			printf("can't be solved\n");
		}
		else if (solve == 1 && euler==1 && t_dist <= 2*3.14159265359) {
			solve = solve * -1;
			printf("in EULER MODE if the distance is smaller from 2PI the problem can't be solved\n");
		}
		else if (solve == 1)
		{
			printf("dist: %f\n", dist);
			if (dist > 0.3) {
				for (int i = robot_parts - 1; i >= 0; i--) {
					pc = vec3(get_end_point(i - 1));
					pt = vec3(qube_position*vec4(0, 0, 0, 1.f));
					pepc = glm::normalize(pe - pc);
					ptpc = glm::normalize(pt - pc);

					float cos = dot(pepc, ptpc);
					float angleA = glm::acos(cos);
					
					vec3 v3 = glm::normalize(glm::vec3(pepc.y*ptpc.z - pepc.z*ptpc.y, pepc.z*ptpc.x - pepc.x*ptpc.z, pepc.x*ptpc.y - pepc.y*ptpc.x));

					mat4 tm;
					if (euler == 1) {
						vec3 aa_to_e = AAtoEuler(v3, angleA);
						mat4 tm_eu = get_euler_mat(aa_to_e.y, aa_to_e.x, aa_to_e.z);
						
						tm = tm_eu * rotation[i];
					}
					else tm = glm::rotate((float)(angleA / 3.14159265359) * 180 / 15, v3) * rotation[i];
					if (tm[0][0] != tm[0][0]) {
						printf("pc(%d):(%f,%f,%f)\n", i, pc.x, pc.y, pc.z);
						printf("pt:(%f,%f,%f)\n", pt.x, pt.y, pt.z);
						printf("pe:(%f,%f,%f)\n", pe.x, pe.y, pe.z);
						printf("pe-pc:(%f,%f,%f)\n", pepc.x, pepc.y, pepc.z);
						printf("pt-pc:(%f,%f,%f)\n", ptpc.x, ptpc.y, ptpc.z);
						printf("cos= %f\n", cos);
						printf("angle:%f\n", angleA);
						printf("v3:(%f,%f,%f)\n", v3.x, v3.y, v3.z);
						for (int l = 0; l < 4; l++) {
							for (int m = 0; m < 4; m++) {
								printf("%f", rotation[i][l][m]);
								if (m != 3) printf(" , ");
								else printf("\n");
							}
						}
						printf("---------------------------------------------------------------\n");
						tm = glm::rotate((float)(1 / 3.14159265359) * 180 / 15, vec3(1, 1, 1)) * rotation[i];
						
					}
					rotation[i] = tm;
					
				}

			}
			else {
				printf("solved\n");
				vec3 pe = vec3(get_end_point(robot_parts-1));
				vec3 pt = vec3(qube_position*vec4(0, 0, 0, 1.f));
				printf("pt:(%f,%f,%f)\n", pt.x, pt.y, pt.z);
				printf("pe:(%f,%f,%f)\n", pe.x, pe.y, pe.z);
				printf("dist=%f", calc_dist());
				solve = -1;
			}
		}
		
		if (pick == 1) {
			index = -1;
			display.Clear(0.f, 0.f, 0.f, 0.0f);
			for (int i = 0; i < robot_parts; i++) {
				MVP_t = rotation[0] * positions[0];
				for (int j = 1; j < i; j++) {
					MVP_t = MVP_t * positions[j] * rotation[j] * positions[j];
				}
				if (i != 0) MVP_t = MVP_t * positions[i];


				if (i != 0)MVP_t = MVP_t * rotation[i] * positions[i];
				MVP = P * GS * M * MVP_t * RS;


				shader.Bind();
				shader.Update(MVP, M*MVP_t, vec3(0,0,i+1));

				mesh.Draw();

			}

			MVP = P * GS * M * qube_position;

			shader.Bind();
			shader.Update(MVP, M, vec3(opt_pic, robot_parts+1));

			mesh.Draw();

			vec4 a=vec4(1.f);
			glReadPixels(x, DISPLAY_HEIGHT - y, 1, 1, GL_RGBA, GL_FLOAT, &a);

			float t5 = round(1/a.z);
			index =(int) (t5 - 1);

			if (index < -1) index = -1;

			pick = pick * -1;
		}
		else {
			Sleep(3);
		}
		display.Clear(1.0f, 1.f, 1.f, 1.0f);

		tex1.Bind(0);
		for (int j = 0; j < 3; j++) {
			mat4 MVP_a = P * GS * M  * axis_translate * axis_rotation[j] * axis_scale;
			
			shader.Bind();
			shader.Update(MVP_a, axis_rotation[j] ,opt_axis);
			mesh_axis.Draw();
		}

		for (int i = 0; i < robot_parts; i++) {
			MVP_t = rotation[0] * positions[0];
			for (int j = 1; j < i; j++) {
				MVP_t = MVP_t * positions[j] * rotation[j] * positions[j] ;
			}
			if (i!=0) MVP_t = MVP_t * positions[i];

			
			if (i > 0) {
				for (int j = 0; j < 3; j++) {
					mat4 MVP_a = P * GS * M * arm_position* MVP_t * positions[i] * glm::translate(glm::vec3(0.f, 0.f, -2.f)) * axis_translate * axis_rotation[j] * axis_scale;
					shader.Bind();
					
					shader.Update(MVP_a, M * axis_rotation[j], opt_axis);
					mesh_axis.Draw();
				}
			}
			
			if (i != 0)MVP_t = MVP_t  * rotation[i] * positions[i];
			MVP = P * GS * M * arm_position * MVP_t * RS;

			
			shader.Bind();
			shader.Update(MVP, M*MVP_t,opt_arm);

			mesh.Draw();
		}

		MVP = P * GS * M * qube_position * qube_rotation;

		tex2.Bind(0);
		shader.Bind();
		shader.Update(MVP, M*qube_rotation, opt_arm);

		mesh.Draw();
		
		string base = "HW4 | Robot Arm | Gal Meir - 305382137 | ";
		string picking_str = "index[";
		if (index == -1) picking_str = picking_str + "GLOBAL";
		else if (index == robot_parts) picking_str = picking_str + "box";
		else picking_str = picking_str + "arm index " + to_string(index);
		picking_str = picking_str + "] | ";
		string dist_str = "IK solver: ";
		if ((t_dist > 2 * robot_scale * robot_parts) || (euler == 1 && t_dist <= 2 * 3.14159265359)) dist_str += "can't be solved |";
		else dist_str += "dist: " + to_string(dist) + " | ";
		if (index == -1) calc_euler(M);
		else if (index == robot_parts) calc_euler(qube_position);
		else calc_euler(rotation[index]);
		string str_mode;
		if (euler == 1) str_mode = " EULER MODE | ";
		else str_mode = " ANGLE-AXIS MODE | ";

		string title = base + str_mode + picking_str + dist_str + "phi: " + to_string(phi) + " , theta: " + to_string(theta) + " , psi: " + to_string(psi) ;
		glfwSetWindowTitle(display.m_window, title.c_str());
		display.SwapBuffers();

		glfwPollEvents();
	}
	

	return 0;
}

void move_X_right()
{
	if (euler == 1) {
		if (index == -1) M = get_euler_mat(0, 0, 1) * M;
		else if (index == robot_parts) qube_rotation = get_euler_mat(0, 0, 1) * qube_rotation;
		else rotation[index] = get_euler_mat(0, 0, 1) * rotation[index];
	}
	else {
		if (index == -1) M = glm::rotate(1.f, glm::vec3(1, 0, 0))* M;
		else if (index == robot_parts) qube_rotation = glm::rotate(-1.f, glm::vec3(M*glm::vec4(1, 0, 0, 0)))  *  qube_rotation;
		else rotation[index] = glm::rotate(rotation[index], 1.f, glm::vec3(1, 0, 0));
	}
}

void move_X_left()
{
	if (euler == 1) {
		if (index == -1) M = get_euler_mat(0, 0, -1) * M;
		else if (index == robot_parts) qube_rotation = get_euler_mat(0, 0, -1) * qube_rotation;
		else rotation[index] = get_euler_mat(0, 0, -1) * rotation[index];
	}
	else {
		if (index == -1) M = glm::rotate(-1.f, glm::vec3(1, 0, 0)) * M;
		else if (index == robot_parts) qube_rotation = glm::rotate(1.f, glm::vec3(M*glm::vec4(1, 0, 0, 0)))  *  qube_rotation;
		else rotation[index] = glm::rotate(rotation[index], -1.f, glm::vec3(1, 0, 0));
	}
}

void move_Z_right()
{
	if (euler == 1) {
		if (index == -1) M = get_euler_mat(1, 0, 0) * M;
		else if (index == robot_parts) qube_rotation = get_euler_mat(1, 0, 0) * qube_rotation;
		else rotation[index] = get_euler_mat(1, 0, 0) * rotation[index];
	}
	else {
		if (index == -1) M = glm::rotate(1.f, glm::vec3(0, 0, 1)) * M;
		else if (index == robot_parts) qube_rotation = glm::rotate(1.f, glm::vec3(M*glm::vec4(0, 0, 1, 0)))  *  qube_rotation;
		else rotation[index] = glm::rotate(1.f, glm::vec3(0, 0, 1)) * rotation[index];
	}
}

void move_Z_left()
{
	if (euler == 1) {
		
		if (index == -1) M = get_euler_mat(-1, 0, 0) * M;
		else if (index == robot_parts) qube_rotation = get_euler_mat(-1, 0, 0) * qube_rotation;
		else rotation[index] = get_euler_mat(-1, 0, 0) * rotation[index];
	}
	else {
		if (index == -1) M = glm::rotate(-1.f, glm::vec3(0, 0, 1)) * M;
		else if (index == robot_parts) qube_rotation = glm::rotate(-1.f, glm::vec3(M*glm::vec4(0, 0, 1, 0))) *  qube_rotation;
		else rotation[index] = glm::rotate(-1.f, glm::vec3(0, 0, 1)) * rotation[index];
	}
}

void index_up()
{
	if (index < robot_parts - 1) index++;
	printf("%d\n", index);
}

void index_down()
{
	if (index > - 1) index--;
	printf("%d\n", index);
}

void flip_solve()
{
	if (glm::length(vec3(qube_position*vec4(1.f))) > 2 * robot_scale*robot_parts) solve = -1;
	else solve = solve * -1;
}

void restart()
{
	solve = -1;
	M = glm::rotate(-90.0f, vec3(1, 0, 0));
	index = -1;
	for (int i = 0; i < robot_parts; i++) {
		rotation[i] = glm::mat4(1.f);
	}
	arm_position = mat4(1.f);
	qube_position = glm::translate(glm::vec3(5.f, 0.f, 0.f));
	qube_rotation = mat4(1.f);
}

void picking(int pos_x, int pos_y)
{
	pick = pick * -1;
	x = pos_x;
	y = pos_y;
}

void move_up()
{
	mat4 tran = glm::translate(glm::vec3( M * glm::vec4(0.f, -1.f, 0.f, 0.f)));
	if (index == robot_parts || index == -1) qube_position = qube_position * tran;
	if (robot_parts>index && index>=-1) arm_position = arm_position * tran;

}

void move_down()
{
	mat4 tran = glm::translate(glm::vec3( M * glm::vec4(0.f, 1.f, 0.f,0.f)));
	if (index == robot_parts || index == -1) qube_position = qube_position * tran;
	if (robot_parts > index && index >= -1) arm_position = arm_position * tran;
}

void move_left()
{
	mat4 tran = glm::translate(glm::vec3(M * glm::vec4(-1.f, 0.f, 0.f, 0.f)));
	if (index == robot_parts || index == -1) qube_position = qube_position * tran;
	if (robot_parts > index && index >= -1) arm_position = arm_position * tran;
}

void move_right()
{
	mat4 tran = glm::translate(glm::vec3(M * glm::vec4(1.f, 0.f, 0.f, 0.f)));
	if (index == robot_parts || index == -1) qube_position = qube_position * tran;
	if (robot_parts > index && index >= -1) arm_position = arm_position * tran;
}

void move_in()
{
	mat4 tran = glm::translate(glm::vec3(M * glm::vec4(0.f, 0.f, -1.f, 0.f)));
	if (index == robot_parts || index == -1) qube_position = qube_position * tran;
	if (robot_parts > index && index >= -1) arm_position = arm_position * tran;
}

void move_out()
{
	mat4 tran = glm::translate(glm::vec3(M * glm::vec4(0.f, 0.f, 1.f, 0.f)));
	if (index == robot_parts || index == -1) qube_position = qube_position * tran;
	if (robot_parts > index && index >= -1) arm_position = arm_position * tran;
}

void flip_euler()
{
	euler = euler * -1;
}

void print_M()
{
	printf("--------------------------------------------------------\n");
	printf("global matrix rotation:\n");
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (M[i][j] >= 0) printf(" ");
			printf("%f ", M[i][j]);
		}
		printf("\n");
	}
	printf("--------------------------------------------------------\n");
}


