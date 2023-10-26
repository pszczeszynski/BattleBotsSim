#pragma comment(lib, "legacy_stdio_definitions.lib") // necessary in release builds for imgui to work

#include "RobotControllerGUI.h"
#include "../RobotConfig.h"
#include "../GuiUtils.h"
#include "../RobotController.h"

RobotControllerGUI::RobotControllerGUI()
{
    // 1. Setup window
    window = SetupWindow();

    // 2. Setup ImGui
    SetupIMGUI(window);
    ImGuiIO &io = ImGui::GetIO();

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
}

bool RobotControllerGUI::Update()
{
    if (glfwWindowShouldClose(window))
    {
        return false;
    }
    // Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
    // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
    glfwPollEvents();

    // setup the ImGui frame
    InitializeImGUIFrame();

    ClearLastFrameTextures();
    // render all ui elements
    FieldWidget::GetInstance().Draw();
    _imuWidget.Draw();
    _configWidget.Draw();
    _robotTelemetryWidget.Draw();


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
 * @brief SetupWindow
 * Initializes a GLFW window and returns a pointer to it.
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
    // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    // glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // 3.0+ only

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
 * @brief SetupIMGUI
 * Initializes the ImGui context and sets up the ImGui style.
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
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Docking
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;   // Enable Multi-Viewport / Platform Windows
    // io.ConfigViewportsNoAutoMerge = true;
    // io.ConfigViewportsNoTaskBarIcon = true;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // set window background to black

    // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
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

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    // - Our Emscripten build process allows embedding fonts to be accessible at runtime from the "fonts/" folder. See Makefile.emscripten for details.
    // io.Fonts->AddFontDefault();
    // io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
    // io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    // io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    // io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    // ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());
    // IM_ASSERT(font != nullptr);
    //ImFont *customFont = io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\consolab.ttf", 15.0f);
    // terminal

    // ImGui::PushFont(customFont);
}

/**
 * @brief InitializeImGUIFrame
 * Initializes the ImGui frame. Should be called every frame,
 * before any ImGui elements are rendered.
 */
void RobotControllerGUI::InitializeImGUIFrame()
{
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

/**
 * @brief Render
 * Renders the ImGui UI.
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

    // Update and Render additional Platform Windows
    // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
    //  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        GLFWwindow *backup_current_context = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwMakeContextCurrent(backup_current_context);
    }

    glfwSwapBuffers(window);
}