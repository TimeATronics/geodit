/* 
 * This file is part of the Geodit distribution (https://github.com/TimeATronics/geodit).
 * Copyright (c) 2024 Aradhya Chakrabarti
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <tiff.h>
#include <tiffio.h>

#include <ios>
#include <iostream>
#include <cstdlib>
#include <string>
#include <iomanip>
#include <sstream>

#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "ImGuiFileDialog.h"

#define GL_SILENCE_DEPRECATION
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <gdal/gdal_priv.h>
#include <gdal/ogr_core.h>
#include <gdal/ogr_spatialref.h>


static bool file_open_opt = false;
static bool exit_opt = false;
static bool zoom_in_opt = false;
static bool zoom_out_opt = false;
static bool status = false;
static bool file_status = false;
std::string filename = "";

double* pixel2coord(int x, int y, const char* filename) {
    const char* pszFilename = filename;
    GDALDatasetUniquePtr poDataset;
    GDALAllRegister();
    const GDALAccess eAccess = GA_ReadOnly;
    
    poDataset = GDALDatasetUniquePtr(GDALDataset::FromHandle(GDALOpen( pszFilename, eAccess )));
    if (!poDataset) {
        printf("Open failed.\n");
        exit(EXIT_FAILURE);
    }
    static double out[2]; double  GT[6];
    if (poDataset->GetGeoTransform(GT) == CE_None){
    	;
    }
    double xoff = GT[0];
    double a = GT[1];
    double b = GT[2];
    double yoff = GT[3];
    double d = GT[4];
    double e = GT[5];
    double xp = a * x + b * y + xoff;
    double yp = d * x + e * y + yoff;
    out[0] = xp; out[1] = yp;

    OGRSpatialReference source = OGRSpatialReference();
    source.importFromWkt(poDataset->GetProjectionRef());
    OGRSpatialReference target = OGRSpatialReference();
    target.importFromEPSG(4326);
    OGRCoordinateTransformation* coord = OGRCreateCoordinateTransformation(&source, &target);
    coord->SetEmitErrors(true);
    if (!coord) {
        return out;
    } else {
        double m = xp; double n = yp;
        coord->Transform(1, &m, &n, nullptr, nullptr);
        static double out1[2];
        out1[0] = m; out1[1] = n;
        return out1;
    }
}

static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int* out_width, int* out_height) {
    int image_width = 0;
    int image_height = 0;
    TIFFRGBAImage img;
	uint32_t *raster;
	size_t npixels;
	TIFF *tif;
  	char emsg[1024];

	tif = TIFFOpen(filename, "r");
  	if (tif == NULL) {
    	fprintf(stderr, "tif == NULL\n");
    	exit(EXIT_FAILURE);
	}
	if (TIFFRGBAImageBegin(&img, tif, 0, emsg)) {
    	npixels = img.width * img.height;
    	raster = (uint32_t *)_TIFFmalloc(npixels * sizeof(uint32_t));
    	if (raster != NULL) {
      		if (TIFFRGBAImageGet(&img, raster, img.width, img.height) == 0) {
      			TIFFError(filename, "%s", emsg);
      			exit(EXIT_FAILURE);
      		}
      	}
      	TIFFRGBAImageEnd(&img);
      	fprintf(stderr, "Read image %s (%d x %d)\n", filename, img.width, img.height);
    } else {
    	TIFFError(filename, "%s", emsg);
    	exit(EXIT_FAILURE);
    }
    image_width = img.width;
    image_height = img.height;
    TIFFClose(tif);
	GLuint image_texture;
	glGenTextures(1, &image_texture);
	glBindTexture(GL_TEXTURE_2D, image_texture);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, raster);

    *out_texture = image_texture;
    *out_width = image_width;
    *out_height = image_height;
    return true;
}

void showDiag() {
	IGFD::FileDialogConfig config;
	config.path = ".";
	ImGuiFileDialog::Instance()->OpenDialog("ChooseFileDlgKey", "Choose GeoTIFF File", ".tif", config);
	if (ImGuiFileDialog::Instance()->Display("ChooseFileDlgKey")) {
    if (ImGuiFileDialog::Instance()->IsOk()) {
      filename = ImGuiFileDialog::Instance()->GetFilePathName();
    }
    ImGuiFileDialog::Instance()->Close();
    file_open_opt = false;
    file_status = true;
  }
}

int main(int, char**) {
/*******************************************************************************************/
    // ImGui setup
	glfwSetErrorCallback([](int error, const char* description) {
		fprintf(stderr, "Glfw Error %d: %s\n", error, description);
	});
	glfwInit();

	const char* glsl_version = "#version 130";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

	GLFWwindow* window = glfwCreateWindow(1280, 720, "Geodit", nullptr, nullptr);
	if(window == NULL) {
		fprintf(stderr, "Failed to create GLFW window\n");
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	glewInit();
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
	io.ConfigWindowsMoveFromTitleBarOnly = true;
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glsl_version);
	int display_w, display_h;
	glfwGetFramebufferSize(window, &display_w, &display_h);

/*******************************************************************************************/
	ImVec2 scroll = ImVec2(0,0);
	ImVec2 cursorPos = ImVec2(0,0);
	ImVec2 imageSize = ImVec2(0.0, 0.0);
	GLuint image_texture;
	float imageScale = 1.0f;
	int image_width = 0;
	int image_height = 0;
	int mouse_x = 0; int mouse_y = 0;

/*******************************************************************************************/
	// Main Loop
	while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();
		//printf("%s\n", filename.c_str());
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		ImGuiViewport * viewport = ImGui::GetMainViewport();
		ImGui::SetCurrentViewport(nullptr, (ImGuiViewportP*)viewport);
		float height = ImGui::GetFrameHeight();
		
		if(ImGui::BeginMainMenuBar()) {
			if(ImGui::BeginMenu("File")) {
				ImGui::MenuItem("Open File", NULL, &file_open_opt);
				ImGui::MenuItem("Exit", NULL, &exit_opt);
				ImGui::EndMenu();
			}
			if(ImGui::BeginMenu("View")) {
				ImGui::MenuItem("Zoom in", NULL, &zoom_in_opt);
				ImGui::MenuItem("Zoom out", NULL, &zoom_out_opt);
				ImGui::EndMenu();
			}
			ImGui::EndMainMenuBar();
		}

		ImGuiID mainDockSpaceId = ImGui::DockSpaceOverViewport();
		ImGuiWindowClass window_class;
		ImGui::SetNextWindowClass(&window_class);
		ImGui::SetNextWindowScroll(scroll);

		if (exit_opt) {
			ImGui_ImplOpenGL3_Shutdown();
			ImGui_ImplGlfw_Shutdown();
			ImGui::DestroyContext();
			glfwDestroyWindow(window);
			glfwTerminate();
			exit(EXIT_SUCCESS);
		}
		if (!status) {
			filename = "";
		}

		if (filename == "" || file_open_opt && !file_status) {
			showDiag();
		}
		     
		if (filename != "" && !file_open_opt) {
			if (!LoadTextureFromFile(filename.c_str(), &image_texture, &image_width, &image_height)) {
				printf("Error creating texture from file.\n");
				exit(EXIT_FAILURE);
			}
			imageSize = ImVec2(imageScale * image_width, imageScale * image_height);
			status = true;
		}

		if (status) {

			ImGui::Begin("Geodit", &status, ImGuiWindowFlags_AlwaysVerticalScrollbar | ImGuiWindowFlags_AlwaysHorizontalScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
			ImGui::GetStyle().WindowPadding.x = 0;
			ImGui::GetStyle().WindowPadding.y = 0;

			imageSize = ImVec2(imageScale * image_width, imageScale * image_height);
			cursorPos.x = (ImGui::GetWindowSize().x - imageSize.x) * 0.5f;
			cursorPos.y = (ImGui::GetWindowSize().y - imageSize.y) * 0.5f;
	    	if (cursorPos.x < 0) cursorPos.x = 0; if (cursorPos.y < 0) cursorPos.y = 0;
			ImGui::SetCursorPos(cursorPos);

			ImGui::Image((void*)(intptr_t)image_texture, imageSize);
			file_open_opt = true;
			ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);

			scroll.x = ImGui::GetScrollX(); scroll.y = ImGui::GetScrollY();

			std::stringstream zoomstream;
			zoomstream << std::fixed << std::setprecision(2) << imageScale*100 << "%";
			std::string zoom_s = zoomstream.str();
			std::string zoomstring = "Zoom: " + zoom_s;

			double* loc = pixel2coord((int)mouse_x, (int)mouse_y, filename.c_str());

			std::stringstream locstream;
			locstream << std::fixed << std::setprecision(5) << loc[0] << "," << loc[1];
			std::string loc_s = locstream.str();
			std::string locstring = "Location: " + loc_s;

			if(ImGui::BeginViewportSideBar("StatusBar", viewport, ImGuiDir_Down, height, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar)) {
				if(ImGui::BeginMenuBar()) {
					ImGui::Text("%s", zoomstring.c_str());
					ImGui::SameLine();
					ImGui::Text("%s", locstring.c_str());
					ImGui::EndMenuBar();
				}
				ImGui::End();
			}
			if (ImGui::IsItemHovered()) {
				ImVec2 mousePosInImage;
				mousePosInImage.x = (ImGui::GetIO().MousePos.x - ImGui::GetWindowPos().x - cursorPos.x + ImGui::GetScrollX()) / imageScale;
				mousePosInImage.y = (ImGui::GetIO().MousePos.y - ImGui::GetWindowPos().y - cursorPos.y + ImGui::GetScrollY()) / imageScale;
				mouse_x = mousePosInImage.x;
				mouse_y = mousePosInImage.y;
				// Check for Ctrl + Scroll
				if (ImGui::GetIO().MouseWheel != 0 && ImGui::GetIO().KeyCtrl) {
					imageScale = imageScale * pow(1.50, ImGui::GetIO().MouseWheel);
					if(imageScale < 0.1) imageScale = 0.1;
					if(imageScale < 0.9) imageScale = 0.9;
					if(imageScale > 30)  imageScale = 30;

					imageSize = ImVec2(imageScale * image_width, imageScale * image_height);
					cursorPos.x = (ImGui::GetWindowSize().x - imageSize.x) * 0.5f;
					cursorPos.y = (ImGui::GetWindowSize().y - imageSize.y) * 0.5f;
					if (cursorPos.x < 0) cursorPos.x = 0; if (cursorPos.y < 0) cursorPos.y = 0;

					scroll.x = mousePosInImage.x * imageScale + ImGui::GetWindowPos().x + cursorPos.x - ImGui::GetIO().MousePos.x;
					scroll.y = mousePosInImage.y * imageScale + ImGui::GetWindowPos().y + cursorPos.y - ImGui::GetIO().MousePos.y;
				}
				// Check for mouse panning
				if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f)) {
					scroll.x = ImGui::GetScrollX() - ImGui::GetIO().MouseDelta.x;
					scroll.y = ImGui::GetScrollY() - ImGui::GetIO().MouseDelta.y;
				}
			}
			ImGui::End();
		}

		ImGui::SetNextWindowClass(&window_class);
		ImGui::Render();
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window);
	}

/*******************************************************************************************/
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
/*******************************************************************************************/
}