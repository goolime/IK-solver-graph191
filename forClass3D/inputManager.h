#pragma once   //maybe should be static class
#include "GLFW\glfw3.h"
#include "main.h"

static const int red1 = 3;
static const int red2 = 5;
static double old_x, old_y, scrol_y;
static bool right_button = false;
static bool left_button = false;


	void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
	{
		switch (key)
		{
		case GLFW_KEY_ESCAPE:
			if(action == GLFW_PRESS)
				glfwSetWindowShouldClose(window,GLFW_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			move_X_right();
			break;
		case GLFW_KEY_LEFT:
			move_X_left();
			break;
		case GLFW_KEY_UP:
			move_Z_right();
			break;
		case GLFW_KEY_DOWN:
			move_Z_left();
			break;
		case GLFW_KEY_A:
			if (action == GLFW_PRESS) index_up();
			break;
		case GLFW_KEY_Z:
			if (action == GLFW_PRESS) index_down();
			break;
		case GLFW_KEY_SPACE:
			if (action == GLFW_PRESS) flip_solve();
			break;
		case GLFW_KEY_X:
			if (action == GLFW_PRESS) restart();
			break;
		case GLFW_KEY_E:
			if (action == GLFW_PRESS) flip_euler();
			break;
		case GLFW_KEY_P:
			if (action == GLFW_PRESS) print_M();
			break;
		default:
			break;
		}
		
	}

	void mouse_callback(GLFWwindow* window, int button, int action, int mods)
	{
		if (button == GLFW_MOUSE_BUTTON_LEFT) {
			if (action == GLFW_PRESS) {
				left_button = true;
			}
			if (action == GLFW_RELEASE) {
				left_button = false;
			}
		}

		if (button == GLFW_MOUSE_BUTTON_RIGHT) {
			if (action == GLFW_PRESS) {
				right_button = true;
			}
			if (action == GLFW_RELEASE) {
				right_button = false;
			}
		}

		if (action == GLFW_PRESS && left_button && right_button) {
			picking(old_x,old_y);
		}
	}

	static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
	{
		
		if (left_button && !right_button) {
			if (xpos < old_x) {
				int tmp = int((old_x - xpos) / red1);
				for (int i = 0; i < tmp; i++) {
					move_Z_left();
				}
			}
			if (xpos > old_x) {
				int tmp = int((xpos - old_x) / red1);
				for (int i = 0; i < tmp; i++) {
					move_Z_right();
				}
			}
			if (ypos < old_y) {
				int tmp = int((old_y - ypos) / red1);
				for (int i = 0; i < tmp; i++) {
					move_X_right();
				}
			}
			if (ypos > old_y) {
				int tmp = int((ypos - old_y) / red1);
				for (int i = 0; i < tmp; i++) {
					move_X_left();
				}
			}
		}
		
		if (right_button && !left_button) {
			if (xpos < old_x) {
				int tmp = (old_x - xpos) / red2;
				for (int i = 0; i < tmp; i++) {
					move_right();
				}
			}
			if (xpos > old_x) {
				int tmp = (xpos - old_x) / red2;
				for (int i = 0; i < tmp; i++) {
					move_left();
				}
			}
			if (ypos < old_y) {
				int tmp = (old_y - ypos) / red2;
				for (int i = 0; i < tmp; i++) {
					move_up();
				}
			}
			if (ypos > old_y) {
				int tmp = (ypos - old_y) / red2;
				for (int i = 0; i < tmp; i++) {
					move_down();
				}
			}
			
		}
		
		old_x = xpos;
		old_y = ypos;
	}

	void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
		
		if (yoffset > 0) move_in();
		if (yoffset < 0) move_out();
		
	}


