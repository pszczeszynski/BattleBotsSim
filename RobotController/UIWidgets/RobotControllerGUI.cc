#pragma comment(lib, "legacy_stdio_definitions.lib") // necessary in release builds for imgui to work

#include "RobotControllerGUI.h"
#include "../RobotConfig.h"
#include "../GuiUtils.h"
#include "../RobotController.h"
// #include <algorithm>

RobotControllerGUI::RobotControllerGUI()
{
    // 1. Setup window
    window = SetupWindow();

    // 2. Setup ImGui
    SetupIMGUI(window);
    ImGuiIO &io = ImGui::GetIO();

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
}

RobotControllerGUI& RobotControllerGUI::GetInstance()
{
    static RobotControllerGUI instance;
    return instance;
}

bool RobotControllerGUI::Update()
{
    if (glfwWindowShouldClose(window))
    {
        return false;
    }

    // Poll and handle events
    glfwPollEvents();
    // setup the ImGui frame
    InitializeImGUIFrame();
    // make sure to clear the last frame's textures
    ClearLastFrameTextures();

    // draw all the widgets
    for (ImageWidget* widget : ImageWidget::Instances())
    {
        widget->Draw();
    }

    _configWidget.Draw();
    _robotTelemetryWidget.Draw();
    _killWidget.Draw();

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    // render the ImGui frame
    Render(window, clear_color);

    return true;
}

void RobotControllerGUI::Shutdown()
{
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}

/**
 * @brief SetupWindow Initializes a GLFW window and returns a pointer to it.
 * @return GLFWwindow* pointer to the GLFW window
 */
GLFWwindow *RobotControllerGUI::SetupWindow()
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
    {
        // return failure
        return nullptr;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+
    // only glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // 3.0+ only

    // Create window with graphics context
    GLFWwindow *window = glfwCreateWindow(IMGUI_WIDTH, IMGUI_HEIGHT, "Dear ImGui GLFW+OpenGL3 example", nullptr, nullptr);
    if (window == nullptr)
    {
        // return failure
        return nullptr;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(IMGUI_ENABLE_VSYNC); // Enable vsync

    return window;
}

/**
 * @brief SetupIMGUI Initializes the ImGui context and sets up the ImGui style.
 * @param window pointer to the GLFW window
 */
void RobotControllerGUI::SetupIMGUI(GLFWwindow *window)
{
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Docking
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;   // Enable Multi-Viewport / Platform Windows
    // io.ConfigViewportsNoAutoMerge = true; io.ConfigViewportsNoTaskBarIcon =
    // true;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // set window background to black

    // When viewports are enabled we tweak WindowRounding/WindowBg so platform
    // windows can look identical to regular ones.
    ImGuiStyle &style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
        style.Colors[ImGuiCol_WindowBg].x = 0.0f;
        style.Colors[ImGuiCol_WindowBg].y = 0.0f;
        style.Colors[ImGuiCol_WindowBg].z = 0.0f;
    }

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    // GL 3.0 + GLSL 130
    const char *glsl_version = "#version 130";

    ImGui_ImplOpenGL3_Init(glsl_version);

    ImFont *customFont = io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\consolab.ttf", 15.0f);
    // terminal

    // ImGui::PushFont(customFont);
}

/**
 * @brief InitializeImGUIFrame Initializes the ImGui frame. Should be called
 * every frame, before any ImGui elements are rendered.
 */
void RobotControllerGUI::InitializeImGUIFrame()
{
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

/**
 * @brief Render Renders the ImGui UI.
 * @param window pointer to the GLFW window
 * @param clearColor color to clear the window with
 */
void RobotControllerGUI::Render(GLFWwindow *window, ImVec4 clearColor)
{
    ImGuiIO &io = ImGui::GetIO();

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clearColor.x * clearColor.w, clearColor.y * clearColor.w, clearColor.z * clearColor.w, clearColor.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Update and Render additional Platform Windows (Platform functions may
    // change the current OpenGL context, so we save/restore it to make it
    // easier to paste this code elsewhere. For this specific demo app we could
    // also call glfwMakeContextCurrent(window) directly)
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        GLFWwindow *backup_current_context = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwMakeContextCurrent(backup_current_context);
    }

    glfwSwapBuffers(window);
}