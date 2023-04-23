#include "Application2D.h"
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Utility/Arguments.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/ConfigurationValue.h>
#include <Magnum/Math/DualComplex.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Square.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/Shaders/FlatGL.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/ImGuiIntegration/Context.hpp>

#include <vector>
#include <memory>

void setup();
void draw(); 

using namespace Magnum;
using namespace Math::Literals;

struct InstanceData
{
    Matrix3 transformation;
    Color3 color;
};

struct DrawMesh
{
    DrawMesh(const Magnum::Trade::MeshData& data)
    {
        instanceBuffer = GL::Buffer{};
        mesh = MeshTools::compile(data);
        mesh.addVertexBufferInstanced(instanceBuffer, 1, 0, Shaders::FlatGL2D::TransformationMatrix{}, Shaders::FlatGL2D::Color3{});
    }

    void Draw(Magnum::Shaders::Flat2D& shader, Math::Matrix3<float>& projection)
    {
        if (instanceData.empty())
            return;

        instanceBuffer.setData({ instanceData.data(), instanceData.size() }, GL::BufferUsage::DynamicDraw);
        mesh.setInstanceCount(instanceData.size());
        shader.setTransformationProjectionMatrix(projection).draw(mesh);
    }

    GL::Mesh mesh{ NoCreate };
    GL::Buffer instanceBuffer{ NoCreate };
    std::vector<InstanceData> instanceData;
};

class MyApplication;
MyApplication* g_application;

class MyApplication : public Platform::Application
{
public:
    explicit MyApplication(const Arguments& arguments);

    void drawEvent() override;
    void viewportEvent(ViewportEvent& event) override;
    void keyPressEvent(KeyEvent& event) override;
    void keyReleaseEvent(KeyEvent& event) override;
    void mousePressEvent(MouseEvent& event) override;
    void mouseReleaseEvent(MouseEvent& event) override;
    void mouseMoveEvent(MouseMoveEvent& event) override;
    void mouseScrollEvent(MouseScrollEvent& event) override;
    void textInputEvent(TextInputEvent& event) override;

    Shaders::FlatGL2D m_shader{ NoCreate };

    DrawMesh m_circle;
    DrawMesh m_circleOutline;
    DrawMesh m_rectanle;
    DrawMesh m_rectanleOutline;

    Math::Matrix3<float> m_cameraProjection;
    Math::Vector2<float> m_cameraCenter;
    Math::Vector2<float> m_cameraSize;
    Math::Vector2<float> m_windowSize;

    ImGuiIntegration::Context g_imgui{ NoCreate };
    void imguiInit();
    void imguiDrawBegin();
    void imguiDrawEnd();

    bool m_mousePressedOld = false;
    bool m_mousePressedNew = false;
};

MyApplication::MyApplication(const Arguments& arguments)
    : Platform::Application{arguments}, 
      m_circle(Primitives::circle2DSolid(60)),
      m_circleOutline(Primitives::circle2DWireframe(60)),
      m_rectanle(Primitives::squareSolid()),
      m_rectanleOutline(Primitives::squareWireframe())
{
#if !defined(CORRADE_TARGET_EMSCRIPTEN) && !defined(CORRADE_TARGET_IOS)
    setWindowTitle("Application2D");
#endif
    g_application = this;

    setSwapInterval(0);
    
    m_windowSize = { 800.0f, 600.0f };
    m_cameraSize = { 20.0f, 15.0f };
    m_cameraProjection = Matrix3::projection({ 20.0f, 15.0f });

    /* Create an instanced shader */
    auto shaderConfig = Shaders::FlatGL2D::Configuration{};
    shaderConfig.setFlags(Shaders::FlatGL2D::Flag::VertexColor | Shaders::FlatGL2D::Flag::InstancedTransformation );
    m_shader = Shaders::FlatGL2D{ shaderConfig };

    setSwapInterval(1);
#if !defined(CORRADE_TARGET_EMSCRIPTEN) && !defined(CORRADE_TARGET_ANDROID)
    setMinimalLoopPeriod(16);
#endif

    setup();

    imguiInit();
}

void MyApplication::drawEvent()
{
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color);

    imguiDrawBegin();

    m_circle.instanceData.clear();
    m_circleOutline.instanceData.clear();
    m_rectanle.instanceData.clear();
    m_rectanleOutline.instanceData.clear();
    
    draw();

    m_circle.Draw(m_shader, m_cameraProjection);
    m_circleOutline.Draw(m_shader, m_cameraProjection);
    m_rectanle.Draw(m_shader, m_cameraProjection);
    m_rectanleOutline.Draw(m_shader, m_cameraProjection);
    
    imguiDrawEnd();

    swapBuffers();
    redraw();

    m_mousePressedOld = m_mousePressedNew;
}

void MyApplication::imguiInit()
{
    g_imgui = ImGuiIntegration::Context(Vector2{ windowSize() } / dpiScaling(), windowSize(), framebufferSize());

    GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add, GL::Renderer::BlendEquation::Add);
    GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha, GL::Renderer::BlendFunction::OneMinusSourceAlpha);

    GL::Renderer::enable(GL::Renderer::Feature::Blending);
    GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);
}

void MyApplication::imguiDrawBegin()
{
    g_imgui.newFrame();
    if (ImGui::GetIO().WantTextInput && !isTextInputActive())
        startTextInput();
    else if (!ImGui::GetIO().WantTextInput && isTextInputActive())
        stopTextInput();
}

void MyApplication::imguiDrawEnd()
{
   g_imgui.updateApplicationCursor(*this);
   g_imgui.drawFrame();
}

void MyApplication::viewportEvent(ViewportEvent& event)
{
    GL::defaultFramebuffer.setViewport({ {}, event.framebufferSize() });

    g_imgui.relayout(Vector2{ event.windowSize() } / event.dpiScaling(), event.windowSize(), event.framebufferSize());
}


void MyApplication::keyPressEvent(KeyEvent& event)
{
    if (g_imgui.handleKeyPressEvent(event)) return;
}

void MyApplication::keyReleaseEvent(KeyEvent& event)
{
    if (g_imgui.handleKeyReleaseEvent(event)) return;
}

void MyApplication::mousePressEvent(MouseEvent& event)
{
    if (g_imgui.handleMousePressEvent(event)) return;

    m_mousePressedNew = true;
}

void MyApplication::mouseReleaseEvent(MouseEvent& event)
{
    if (g_imgui.handleMouseReleaseEvent(event)) return;

    m_mousePressedNew = false;
}

void MyApplication::mouseMoveEvent(MouseMoveEvent& event) 
{
    if (g_imgui.handleMouseMoveEvent(event)) return;
}

void MyApplication::mouseScrollEvent(MouseScrollEvent& event) 
{
    if (g_imgui.handleMouseScrollEvent(event)) 
    {
        /* Prevent scrolling the page */
        event.setAccepted();
        return;
    }
}

void MyApplication::textInputEvent(TextInputEvent& event)
{
    if (g_imgui.handleTextInputEvent(event)) return;
}

Math::Matrix3<float> CreateTransformation(Vector2 translation, float radians, Vector2 scale)
{
    auto rotation = Math::Complex<float>::rotation(Math::Rad<float>(radians));

    return Math::Matrix3<float>::from(rotation.toMatrix(), translation) * Math::Matrix3<float>::scaling(scale);
}

namespace Application2D
{
    Color rgb(uint8_t r, uint8_t g, uint8_t b)
    {
        return Color(r / 255.0f, g / 255.0f, b / 255.0f);
    }

    void setCameraCenter(Point center)
    {
        g_application->m_cameraCenter = center;
        g_application->m_cameraProjection = Math::Matrix3<float>::projection(g_application->m_cameraCenter - g_application->m_cameraSize / 2.0f, 
                                                                             g_application->m_cameraCenter + g_application->m_cameraSize / 2.0f);
    }

    Point getCameraCenter()
    {
        return g_application->m_cameraCenter;
    }

    void setCameraSize(Point size)
    {
        g_application->m_cameraSize = size;
        g_application->m_cameraProjection = Math::Matrix3<float>::projection(g_application->m_cameraCenter - g_application->m_cameraSize / 2.0f,
                                                                             g_application->m_cameraCenter + g_application->m_cameraSize / 2.0f);
    }

    Point getCameraSize()
    {
        return g_application->m_cameraSize;
    }

    Point getMousePositionWindow()
    {
        int x = 0, y = 0;

        SDL_GetMouseState(&x, &y);

        y = g_application->windowSize().y() - y;

        return { (float)x, (float)y };
    }

    Point convertCameraToWindow(const Point& p)
    {
        Point result = p + g_application->m_cameraSize / 2.0f - g_application->m_cameraCenter;

        result.x() *= (g_application->m_windowSize.x() / g_application->m_cameraSize.x());
        result.y() *= (g_application->m_windowSize.y() / g_application->m_cameraSize.y());

        return result;
    }

    Point convertWindowToCamera(const Point& p)
    {
        Point result = -g_application->m_cameraSize / 2.0f;
        
        result.x() += (p.x() / g_application->m_windowSize.x()) * g_application->m_cameraSize.x() + g_application->m_cameraCenter.x();
        result.y() += (p.y() / g_application->m_windowSize.y()) * g_application->m_cameraSize.y() + g_application->m_cameraCenter.y();

        return result;
    }

    void drawCircle(Point center, float radius, Color color)
    {
        g_application->m_circle.instanceData.push_back({ CreateTransformation(center, 0.0f, { radius, radius }), color });
    }

    void drawCircleOutline(Point center, float radius, Color color)
    {
        g_application->m_circleOutline.instanceData.push_back({ CreateTransformation(center, 0.0f, { radius, radius }), color });
    }

    void drawRectangle(Point center, float width, float height, Color color)
    {
        g_application->m_rectanle.instanceData.push_back({ CreateTransformation(center, 0.0f, { width, height }), color });
    }

    void drawRectangle(Point center, float rotation, float width, float height, Color color)
    {
        g_application->m_rectanle.instanceData.push_back({ CreateTransformation(center, rotation, { width, height }), color });
    }

    void drawPolygon(const std::vector<Point>& points, Color color)
    {
        GL::Buffer vertices;
        vertices.setData({ points.data(), points.size() }, GL::BufferUsage::StaticDraw);

        GL::Mesh mesh;
        mesh.addVertexBuffer(vertices, 0, Shaders::FlatGL2D::Position{});
        mesh.setCount(points.size());

        Shaders::FlatGL2D shader;
        shader.setColor(color).setTransformationProjectionMatrix(g_application->m_cameraProjection).draw(mesh);
    }

    void drawPolyline(const std::vector<Point>& points, Color color)
    {
        GL::Buffer vertices;
        vertices.setData({ points.data(), points.size() }, GL::BufferUsage::StaticDraw);

        GL::Mesh mesh{ MeshPrimitive::LineStrip };
        mesh.addVertexBuffer(vertices, 0, Shaders::FlatGL2D::Position{});
        mesh.setCount(points.size());

        Shaders::FlatGL2D shader;
        shader.setColor(color).setTransformationProjectionMatrix(g_application->m_cameraProjection).draw(mesh);
    }

    bool isMousePressed()
    {
        return g_application->m_mousePressedNew && g_application->m_mousePressedNew != g_application->m_mousePressedOld;
    }

    bool isMouseReleased()
    {
        return !g_application->m_mousePressedNew && g_application->m_mousePressedNew != g_application->m_mousePressedOld;
    }

    bool isMouseDown()
    {
        return g_application->m_mousePressedNew;
    }
}

MAGNUM_APPLICATION_MAIN(MyApplication)
