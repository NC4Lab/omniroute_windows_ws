// projection_display.cpp

#include "projection_display.h"

// Variables

const int nProjectors = 2;

std::string windowNames[nProjectors] = {
    "Projection Display 1",
    "Projection Display 2",
    "Projection Display 3",
    "Projection Display 4"};

GLFWwindow *p_windowIDVec[nProjectors];
GLuint fboIDVec[nProjectors];
GLuint fboTextureIDVec[nProjectors];

// Functions

void drawWallsAllForWindow(int windowIndex)
{
    drawWallsAll(H, cpParam, fbo_texture_id,
                 imgTestIDVec[imgTestInd],
                 imgMonIDVec[winMonInd],
                 imgParamIDVec[imgParamInd],
                 imgCalIDVec[calModeInd]);
}

int main(int argc, char **argv)
{
        //  _______________ SETUP _______________

    // ROS Initialization
    ros::init(argc, argv, "projection_calibration", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ROS_INFO("RUNNING MAIN");

    GLuint fbo_ids[nProjectors];
    GLuint fbo_texture_ids[nProjectors];

        // --------------- OpenGL SETUP ---------------

    // Initialize GLFW and create windows
    for (int i = 0; i < nProjectors; ++i)
    {
        p_windowIDVec[i] = glfwCreateWindow(
            PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL,
            windowNames[i].c_str(), NULL, NULL);
        if (!p_windowIDVec[i])
        {
            glfwTerminate();
            return -1;
        }

        // Set OpenGL context and callbacks
        glfwMakeContextCurrent(p_windowIDVec[i]);
        gladLoadGL();
        glfwSetKeyCallback(p_windowIDVec[i], callbackKeyBinding);
        glfwSetFramebufferSizeCallback(p_windowIDVec[i],
                                       callbackFrameBufferSizeGLFW);

        // Generate and set up the FBO
        glGenFramebuffers(1, &fbo_ids[i]);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_ids[i]);

        // Generate and set up the texture
        glGenTextures(1, &fbo_texture_ids[i]);
        glBindTexture(GL_TEXTURE_2D, fbo_texture_ids[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Attach the texture to the FBO
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texture_ids[i], 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    // --------------- DevIL SETUP ---------------

    // Initialize DevIL library
    ilInit();

    // Load images
    loadImgTextures(imgWallIDVec, imgWallPathVec);

    // Main loop
    while (true)
    {
        for (int i = 0; i < nProjectors; ++i)
        {
            glfwMakeContextCurrent(p_windowIDVec[i]);
            glClear(GL_COLOR_BUFFER_BIT);

            // Draw walls for this window
            drawWallsAllForWindow(i);

            glfwSwapBuffers(p_windowIDVec[i]);
            glfwPollEvents();

            if (glfwWindowShouldClose(p_windowIDVec[i]))
            {
                // Handle window close
            }
        }
    }

    // Cleanup code

    return 0;
}
