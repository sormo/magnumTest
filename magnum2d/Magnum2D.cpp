#include "Magnum2D.h"
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/Debug.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/ConfigurationValue.h>
#include <Magnum/Math/DualComplex.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/CompileLines.h>
#include <Magnum/MeshTools/GenerateLines.h>
#ifndef CORRADE_TARGET_EMSCRIPTEN
#include <Magnum/Platform/Sdl2Application.h>
#else
#include <Magnum/Platform/EmscriptenApplication.h>
#endif
#include <Magnum/Primitives/Square.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/Shaders/FlatGL.h>
#include <Magnum/Shaders/LineGL.h>
#include <Magnum/Shaders/Line.h>
#include <Magnum/Shaders/DistanceFieldVectorGL.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/ImGuiIntegration/Context.hpp>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Text/AbstractFont.h>
#include <Magnum/Text/GlyphCache.h>
#include <Magnum/Text/Renderer.h>

#include <iostream>
#include <string_view>
#include <vector>
#include <memory>
#include <chrono>
#include <map>
#include <set>

void setup();
void draw();
void drawGui();

using namespace Magnum;
using namespace Math::Literals;

using bytes = std::span<const std::byte>;

template <>
struct std::hash<bytes>
{
    std::size_t operator()(const bytes& x) const noexcept
    {
        return std::hash<std::string_view>{}({ reinterpret_cast<const char*>(x.data()), x.size() });
    }
};

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

    void Draw(Magnum::Shaders::FlatGL2D& shader, Math::Matrix3<float>& projection)
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

bool SigmaCompare(float a, float b)
{
    static const float Sigma = 0.1f;
    return std::fabs(a - b) <= Sigma;
}

struct LineMeshCache
{
    GL::Mesh* Get(size_t hash);
    GL::Mesh* Add(size_t hash, std::span<Magnum2D::vec2> points);
    void Update();

    std::map<size_t, GL::Mesh> cache;
    std::set<size_t> cacheDrawn;
};

struct TextRendererCache
{
    struct Key
    {
        float height;
        UnsignedByte alignment;

        bool operator==(const Key& o)
        {
            return alignment == o.alignment && SigmaCompare(height, o.height);
        }
        bool operator<(const Key& o) const
        {
            return height < o.height;
        }
    };

    Text::Renderer2D* Get(float height, UnsignedByte alignment);
    Text::Renderer2D* Add(float height, UnsignedByte alignment);
    void Update();

    std::map<Key, std::unique_ptr<Text::Renderer2D>> cache;
    std::set<Key> cacheDrawn;
};

class MyApplication : public Platform::Application
{
public:
    static constexpr float CameraSizeFactor = 1.0f / 40.0f; // cameraSize = windowSize * factor

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

    PluginManager::Manager<Trade::AbstractImporter> m_importerManager;
    PluginManager::Manager<Text::AbstractFont> m_fontManager;
    Containers::Pointer<Text::AbstractFont> m_font;
    Containers::Pointer<Text::GlyphCache> m_fontGlyphCache;

    Shaders::FlatGL2D m_shaderInstanced{ NoCreate };
    Shaders::FlatGL2D m_shaderDefault;
    Shaders::LineGL2D m_shaderLine{ NoCreate };
    Shaders::LineGL2D m_shaderCircle{ NoCreate };
    Shaders::DistanceFieldVectorGL2D m_textShader;

    DrawMesh m_circle;
    DrawMesh m_circleOutline;
    DrawMesh m_rectanle;
    DrawMesh m_rectanleOutline;

    Math::Matrix3<float> m_cameraProjection;
    Math::Vector2<float> m_cameraCenter;
    Math::Vector2<float> m_cameraSize;
    Math::Vector2<float> m_windowSize;
    bool m_windowResized = true;

    void setupCamera();
    void setupText();

    ImGuiIntegration::Context g_imgui{ NoCreate };
    void imguiInit();
    void imguiDrawBegin();
    void imguiDrawEnd();

    struct PressedEvent
    {
        bool valueOld = false;
        bool valueNew = false;

        void update()
        {
            valueOld = valueNew;
        }
    };

    Math::Vector2<float> m_mousePosition;
    Math::Vector2<float> m_mouseDelta;
    Math::Vector2<float> m_mouseScroll;

    std::map<Magnum2D::Mouse, PressedEvent> m_mousePressed;
    std::map<char, PressedEvent> m_pressedKeys;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_startApplication;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startFrame;
    float m_frameDeltaMs = 0.0f;

    LineMeshCache m_lineMeshCache;
    TextRendererCache m_textRendererCache;

    // Mesh for circle with width
    GL::Mesh m_circleOutlineLineMesh;
    GL::Mesh m_circleLineMesh;

    Magnum2D::transform m_globalTransform;
    Matrix3x3 m_globalTransformMatrix{ Math::IdentityInit };
};

Platform::Application::Configuration CreateConfiguration()
{
    Platform::Application::Configuration result;

    result.setTitle("Application2D");
    result.addWindowFlags(Platform::Application::Configuration::WindowFlags::Type::Resizable);

    return result;
}

Trade::MeshData CreateCircleLineMeshData()
{
    Trade::MeshAttributeData attr{ Trade::MeshAttribute::Position, VertexFormat::Vector2, 0, 2, sizeof(Vector2) };
    Containers::Array<char> vertexData{ sizeof(Vector2) *2 }; // two points {{0,0}, {0,0}}
    return Trade::MeshData{ MeshPrimitive::Lines, std::move(vertexData), { attr } };
}

MyApplication::MyApplication(const Arguments& arguments)
    : Platform::Application{arguments, CreateConfiguration()},
      m_circle(Primitives::circle2DSolid(60)),
      m_circleOutline(Primitives::circle2DWireframe(60)),
      m_rectanle(Primitives::squareSolid()),
      m_rectanleOutline(Primitives::squareWireframe())
{
#if !defined(CORRADE_TARGET_EMSCRIPTEN) && !defined(CORRADE_TARGET_IOS)
    setSwapInterval(0);
#endif
    g_application = this;

    setupCamera();
    setupText();

    m_startApplication = m_startFrame = std::chrono::high_resolution_clock::now();

    /* Create an instanced shader */
    auto instanceShaderConfig = Shaders::FlatGL2D::Configuration{};
    instanceShaderConfig.setFlags(Shaders::FlatGL2D::Flag::VertexColor | Shaders::FlatGL2D::Flag::InstancedTransformation );
    m_shaderInstanced = Shaders::FlatGL2D{ instanceShaderConfig };

    /* Create a linegl shader */
    auto lineShaderConfig = Shaders::LineGL2D::Configuration{};
    lineShaderConfig.setJoinStyle(Shaders::LineJoinStyle::Bevel);
    m_shaderLine = Shaders::LineGL2D{ lineShaderConfig };

    /* Create n linegl instanced shader */
    auto circleShaderConfig = Shaders::LineGL2D::Configuration{};
    circleShaderConfig.setCapStyle(Shaders::LineCapStyle::Round);
    m_shaderCircle = Shaders::LineGL2D{ circleShaderConfig };

    // Mesh for nice circle outlines
    m_circleOutlineLineMesh = MeshTools::compileLines(MeshTools::generateLines(Primitives::circle2DWireframe(64)));
    // Mesh for nice circles
    m_circleLineMesh = MeshTools::compileLines(MeshTools::generateLines(CreateCircleLineMeshData()));

#if !defined(CORRADE_TARGET_EMSCRIPTEN) && !defined(CORRADE_TARGET_ANDROID)
    setSwapInterval(1);
    setMinimalLoopPeriod(16);
#endif

    setup();

    imguiInit();
}

// sometimes for some reason the resource cpp is not compiled
int resourceInitializer_Magnum2D_RESOURCES();

void MyApplication::setupText()
{
    // for some reason this one may not be called
    resourceInitializer_Magnum2D_RESOURCES();

    m_importerManager.loadAndInstantiate("TgaImporter");
    m_fontManager.registerExternalManager(m_importerManager);

    /* Load MagnumFont plugin */
    m_font = m_fontManager.loadAndInstantiate("MagnumFont");
    if (!m_font)
        std::exit(1);

    m_font->setFileCallback([](const std::string& filename, InputFileCallbackPolicy, void*)
    {
        Utility::Resource rs("fonts");
        return Containers::optional(rs.getRaw(filename));
    });

    /* Open the font and fill glyph cache */
    if (!m_font->openFile("SourceSansPro.conf", 0.0f))
    {
        Error() << "Cannot open font file";
        std::exit(1);
    }
    /* We know it's Text::GlyphCache, so cast it. Sigh, this is awful. */
    m_fontGlyphCache = Containers::pointerCast<Text::GlyphCache>(m_font->createGlyphCache());
    CORRADE_INTERNAL_ASSERT(m_fontGlyphCache);
}

void prepareRendererDraw()
{
    GL::Renderer::enable(GL::Renderer::Feature::Blending);
    GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::One, GL::Renderer::BlendFunction::OneMinusSourceAlpha);
}

void MyApplication::drawEvent()
{
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color);

    m_circle.instanceData.clear();
    m_circleOutline.instanceData.clear();
    m_rectanle.instanceData.clear();
    m_rectanleOutline.instanceData.clear();

    // delta time
    auto now = std::chrono::high_resolution_clock::now();
    m_frameDeltaMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_startFrame).count();
    m_startFrame = now;

    // drawing

    prepareRendererDraw();

    draw();

    m_rectanle.Draw(m_shaderInstanced, m_cameraProjection);
    m_rectanleOutline.Draw(m_shaderInstanced, m_cameraProjection);
    m_circle.Draw(m_shaderInstanced, m_cameraProjection);
    m_circleOutline.Draw(m_shaderInstanced, m_cameraProjection);

    imguiDrawBegin();
    
    drawGui();

    imguiDrawEnd();

    // line cache - clear meshes which were not drawn
    m_lineMeshCache.Update();
    m_textRendererCache.Update();

    m_windowResized = false;

    // input stuff

    for (auto& v : m_mousePressed)
        v.second.update();
    for (auto& v : m_pressedKeys)
        v.second.update();
    m_mouseDelta.x() = 0; m_mouseDelta.y() = 0;
    m_mouseScroll.x() = 0; m_mouseScroll.y() = 0;

    // finish

    swapBuffers();
    redraw();
}

void MyApplication::imguiInit()
{
    g_imgui = ImGuiIntegration::Context(Vector2{ windowSize() } / dpiScaling(), windowSize(), framebufferSize());
}

void MyApplication::imguiDrawBegin()
{
    GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add, GL::Renderer::BlendEquation::Add);
    GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha, GL::Renderer::BlendFunction::OneMinusSourceAlpha);

    GL::Renderer::enable(GL::Renderer::Feature::Blending);
    GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);

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

void MyApplication::setupCamera()
{
    m_windowSize = { (float)windowSize().x(), (float)windowSize().y() };
    m_cameraSize = m_windowSize * CameraSizeFactor;
    m_cameraProjection = Matrix3::projection(m_cameraSize);
}

void MyApplication::viewportEvent(ViewportEvent& event)
{
    GL::defaultFramebuffer.setViewport({ {}, event.framebufferSize() });

    g_imgui.relayout(Vector2{ event.windowSize() } / event.dpiScaling(), event.windowSize(), event.framebufferSize());

    setupCamera();

    m_windowResized = true;
}


void MyApplication::keyPressEvent(KeyEvent& event)
{
    if (g_imgui.handleKeyPressEvent(event)) return;

#if !defined(CORRADE_TARGET_EMSCRIPTEN)
    if (event.isRepeated())
        return;
#endif

    std::cout << "pressed\n";

    m_pressedKeys[(char)event.key()] = { false, true };
}

void MyApplication::keyReleaseEvent(KeyEvent& event)
{
    if (g_imgui.handleKeyReleaseEvent(event)) return;

    m_pressedKeys[(char)event.key()] = { true, false };
}

#if defined(CORRADE_TARGET_EMSCRIPTEN)
Magnum2D::Mouse getMouseTarget(int32_t button)
{
    switch (button)
    {
    case 0: // left
        return Magnum2D::Mouse::left;
    case 1: // middle
        return Magnum2D::Mouse::middle;
    case 2: // right
        return Magnum2D::Mouse::right;
    }
    return Magnum2D::Mouse::left;
}
#else
Magnum2D::Mouse getMouseTarget(int32_t button)
{
    switch (button)
    {
    case 1: // left
        return Magnum2D::Mouse::left;
    case 2: // middle
        return Magnum2D::Mouse::middle;
    case 3: // right
        return Magnum2D::Mouse::right;
    }
    return Magnum2D::Mouse::left;
}
#endif

void MyApplication::mousePressEvent(MouseEvent& event)
{
    if (g_imgui.handleMousePressEvent(event)) return;

    Magnum2D::Mouse target = getMouseTarget((int32_t)event.button());
    m_mousePressed[target].valueNew = true;
}

void MyApplication::mouseReleaseEvent(MouseEvent& event)
{
    if (g_imgui.handleMouseReleaseEvent(event)) return;

    Magnum2D::Mouse target = getMouseTarget((int32_t)event.button());
    m_mousePressed[target].valueNew = false;
}

void MyApplication::mouseMoveEvent(MouseMoveEvent& event) 
{
    Math::Vector2<float> newMousePosition(event.position());

    newMousePosition.y() = g_application->windowSize().y() - newMousePosition.y();

    m_mouseDelta = newMousePosition - m_mousePosition;
    m_mousePosition = newMousePosition;

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

    m_mouseScroll = event.offset();
}

void MyApplication::textInputEvent(TextInputEvent& event)
{
    if (g_imgui.handleTextInputEvent(event)) return;
}

Math::Matrix3<float> CreateTransformation(Vector2 translation, float radians, Vector2 scale)
{
    auto rotation = Math::Complex<float>::rotation(Math::Rad<float>(radians + g_application->m_globalTransform.rotation));

    return Math::Matrix3<float>::from(rotation.toMatrix(), translation + g_application->m_globalTransform.position) * Math::Matrix3<float>::scaling(scale);
}

Text::Renderer2D* TextRendererCache::Get(float height, UnsignedByte alignment)
{
    if (cache.contains({ height, alignment }))
    {
        cacheDrawn.insert({ height, alignment });
        return cache[{ height, alignment }].get();
    }
    return nullptr;
}

Text::Renderer2D* TextRendererCache::Add(float height, UnsignedByte alignment)
{
    cache[{ height, alignment }] = std::make_unique<Magnum::Text::Renderer2D>(*g_application->m_font, *g_application->m_fontGlyphCache, height, (Text::Alignment)alignment);
    cache[{ height, alignment }]->reserve(50, GL::BufferUsage::DynamicDraw, GL::BufferUsage::StaticDraw);
    cacheDrawn.insert({ height, alignment });

    return cache[{ height, alignment }].get();
}

void TextRendererCache::Update()
{
    for (auto it = std::begin(cache); it != std::end(cache);)
    {
        if (!cacheDrawn.contains(it->first))
            it = cache.erase(it);
        else
            it++;
    }
    cacheDrawn.clear();
}

GL::Mesh* LineMeshCache::Get(size_t hash)
{
    if (cache.contains(hash))
    {
        cacheDrawn.insert(hash);
        return &cache[hash];
    }
    return nullptr;
}

Trade::MeshData lineMesh(std::span<Magnum2D::vec2> points, MeshPrimitive primitive)
{
    Containers::Array<char> vertexData{sizeof(Vector2)* points.size()};
    auto positions = Containers::arrayCast<Vector2>(vertexData);

    for (size_t i = 0; i < points.size(); i++)
        positions[i] = points[i];

    Trade::MeshAttributeData attr{ Trade::MeshAttribute::Position, VertexFormat::Vector2, 0, (uint32_t)points.size(), sizeof(Vector2) };

    return Trade::MeshData{primitive, std::move(vertexData), { attr }};
}

Trade::MeshData lineMeshStrip2D(std::span<Magnum2D::vec2> points)
{
    return lineMesh(points, Magnum::MeshPrimitive::LineStrip);
}

Trade::MeshData lineMeshLines(std::span<Magnum2D::vec2> points)
{
    return lineMesh(points, Magnum::MeshPrimitive::Lines);
}

GL::Mesh* LineMeshCache::Add(size_t hash, std::span<Magnum2D::vec2> points)
{
    auto meshData = lineMeshStrip2D(points);
    auto meshDataLines = MeshTools::generateLines(meshData);
    auto meshNew = MeshTools::compileLines(meshDataLines);

    cache[hash] = std::move(meshNew);
    cacheDrawn.insert(hash);

    return &cache[hash];
}

void LineMeshCache::Update()
{
    for (auto it = std::begin(cache); it != std::end(cache);)
    {
        if (!cacheDrawn.contains(it->first))
            it = cache.erase(it);
        else
            it++;
    }
    cacheDrawn.clear();
}

namespace Magnum2D
{
    col3 rgb(uint8_t r, uint8_t g, uint8_t b)
    {
        return col3(r / 255.0f, g / 255.0f, b / 255.0f);
    }

    vec2 getWindowSize()
    {
        return g_application->m_windowSize;
    }

    void setCameraCenter(vec2 center)
    {
        g_application->m_cameraCenter = center;
        g_application->m_cameraProjection = Math::Matrix3<float>::projection(g_application->m_cameraCenter - g_application->m_cameraSize / 2.0f, 
                                                                             g_application->m_cameraCenter + g_application->m_cameraSize / 2.0f);
    }

    vec2 getCameraCenter()
    {
        return g_application->m_cameraCenter;
    }

    void setCameraSize(vec2 size)
    {
        g_application->m_cameraSize = size;
        g_application->m_cameraProjection = Math::Matrix3<float>::projection(g_application->m_cameraCenter - g_application->m_cameraSize / 2.0f,
                                                                             g_application->m_cameraCenter + g_application->m_cameraSize / 2.0f);
    }

    vec2 getCameraSize()
    {
        return g_application->m_cameraSize;
    }

    vec2 getMousePositionWorld()
    {
        return convertWindowToWorld(getMousePositionWindow());
    }

    vec2 getMousePositionWindow()
    {
        return g_application->m_mousePosition;
    }

    vec2 getMouseDeltaWindow()
    {
        return g_application->m_mouseDelta;
    }

    float getMouseScroll()
    {
        return g_application->m_mouseScroll.y();
    }

    vec2 convertCameraToWindow(const vec2& p)
    {
        vec2 result = p + g_application->m_cameraSize / 2.0f - g_application->m_cameraCenter;

        result.x() *= (g_application->m_windowSize.x() / g_application->m_cameraSize.x());
        result.y() *= (g_application->m_windowSize.y() / g_application->m_cameraSize.y());

        return result;
    }

    vec2 convertWindowToWorld(const vec2& p)
    {
        vec2 result = -g_application->m_cameraSize / 2.0f;
        
        result.x() += (p.x() / g_application->m_windowSize.x()) * g_application->m_cameraSize.x() + g_application->m_cameraCenter.x();
        result.y() += (p.y() / g_application->m_windowSize.y()) * g_application->m_cameraSize.y() + g_application->m_cameraCenter.y();

        return result;
    }

    vec2 convertWindowToWorldVector(const vec2& p)
    {
        return { (p.x() / g_application->m_windowSize.x()) * g_application->m_cameraSize.x(), (p.y() / g_application->m_windowSize.y()) * g_application->m_cameraSize.y() };
    }

    void drawCircle(vec2 center, float radius, col3 color)
    {
        g_application->m_circle.instanceData.push_back({ CreateTransformation(center, 0.0f, vec2{ radius, radius }), color });
    }

    void drawCircle2(vec2 center, float radius, col3 color)
    {
        float factor = g_application->m_windowSize.x() / g_application->m_cameraSize.x();
        radius *= 2.0f*factor;

        g_application->m_shaderCircle.setViewportSize(Vector2{ GL::defaultFramebuffer.viewport().size() })
            .setTransformationProjectionMatrix(g_application->m_cameraProjection * CreateTransformation(center, 0.0f, { radius, radius }))
            .setColor(color)
            .setWidth(radius)
            .setSmoothness(radius * MyApplication::CameraSizeFactor)
            .draw(g_application->m_circleLineMesh);
    }

    void drawCircleOutline(vec2 center, float radius, col3 color)
    {
        g_application->m_circleOutline.instanceData.push_back({ CreateTransformation(center, 0.0f, vec2{ radius, radius }), color });
    }

    void drawCircleOutline2(vec2 center, float radius, float width, col3 color)
    {
        //GL::Renderer::enable(GL::Renderer::Feature::Blending);
        //GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::One, GL::Renderer::BlendFunction::OneMinusSourceAlpha);

        float factor = g_application->m_windowSize.x() / g_application->m_cameraSize.x();
        width *= factor;

        g_application->m_shaderLine.setViewportSize(Vector2{ GL::defaultFramebuffer.viewport().size() })
                                   .setTransformationProjectionMatrix(g_application->m_cameraProjection * CreateTransformation(center, 0.0f, { radius, radius }))
                                   .setColor(color)
                                   .setWidth(width)
                                   .setSmoothness(width / 2.0f)
                                   .draw(g_application->m_circleOutlineLineMesh);
    }

    void drawRectangle(vec2 center, float width, float height, col3 color)
    {
        g_application->m_rectanle.instanceData.push_back({ CreateTransformation(center, 0.0f, { width / 2.0f, height / 2.0f }), color });
    }

    void drawRectangle(vec2 center, float rotation, float width, float height, col3 color)
    {
        g_application->m_rectanle.instanceData.push_back({ CreateTransformation(center, rotation * 0.0174533f, { width / 2.0f, height / 2.0f }), color });
    }

    void drawPolygon(const std::vector<vec2>& points, col3 color)
    {
        GL::Buffer vertices;
        vertices.setData({ points.data(), points.size() }, GL::BufferUsage::StaticDraw);

        GL::Mesh mesh{ MeshPrimitive::TriangleFan };
        mesh.addVertexBuffer(vertices, 0, Shaders::FlatGL2D::Position{});
        mesh.setCount(points.size());

        g_application->m_shaderDefault.setColor(color).setTransformationProjectionMatrix(g_application->m_cameraProjection * g_application->m_globalTransformMatrix).draw(mesh);
    }

    void drawPolyline(const std::vector<vec2>& points, col3 color)
    {
        GL::Buffer vertices;
        vertices.setData({ points.data(), points.size() }, GL::BufferUsage::StaticDraw);

        GL::Mesh mesh{ MeshPrimitive::LineStrip };
        mesh.addVertexBuffer(vertices, 0, Shaders::FlatGL2D::Position{});
        mesh.setCount(points.size());

        g_application->m_shaderDefault.setColor(color).setTransformationProjectionMatrix(g_application->m_cameraProjection * g_application->m_globalTransformMatrix).draw(mesh);
    }

    void drawPolyline(std::span<vec2> points, col3 color)
    {
        GL::Buffer vertices;
        vertices.setData({ points.data(), points.size() }, GL::BufferUsage::StaticDraw);

        GL::Mesh mesh{ MeshPrimitive::LineStrip };
        mesh.addVertexBuffer(vertices, 0, Shaders::FlatGL2D::Position{});
        mesh.setCount(points.size());

        g_application->m_shaderDefault.setColor(color).setTransformationProjectionMatrix(g_application->m_cameraProjection * g_application->m_globalTransformMatrix).draw(mesh);
    }

    void drawPolyline2(const std::vector<vec2>& points, float width, col3 color)
    {
        drawPolyline2(std::span<vec2>(const_cast<std::vector<vec2>&>(points)), width, color);
    }

    void drawPolyline2(std::span<vec2> points, float width, col3 color)
    {
        if (points.size() <= 1)
            return;

        auto hasher = std::hash<bytes>{};
        size_t hash = hasher(std::as_bytes(std::span(points)));

        GL::Mesh* mesh = g_application->m_lineMeshCache.Get(hash);

        if (!mesh)
            mesh = g_application->m_lineMeshCache.Add(hash, points);

        float factor = g_application->m_windowSize.x() / g_application->m_cameraSize.x();
        width *= factor;

        g_application->m_shaderLine.setViewportSize(Vector2{ GL::defaultFramebuffer.viewport().size() })
                                   .setTransformationProjectionMatrix(g_application->m_cameraProjection * g_application->m_globalTransformMatrix)
                                   .setColor(color)
                                   .setWidth(width)
                                   .setSmoothness(width / 2.0f)
                                   .draw(*mesh);
    }

    void drawLines(const std::vector<vec2>& points, col3 color)
    {
        GL::Buffer vertices;
        vertices.setData({ points.data(), points.size() }, GL::BufferUsage::StaticDraw);

        GL::Mesh mesh{ MeshPrimitive::Lines };
        mesh.addVertexBuffer(vertices, 0, Shaders::FlatGL2D::Position{});
        mesh.setCount(points.size());

        g_application->m_shaderDefault.setColor(color).setTransformationProjectionMatrix(g_application->m_cameraProjection * g_application->m_globalTransformMatrix).draw(mesh);
    }

    void drawLines2(std::vector<vec2>& points, float width, col3 color)
    {
        auto hasher = std::hash<bytes>{};
        size_t hash = hasher(std::as_bytes(std::span(points)));

        GL::Mesh* mesh = g_application->m_lineMeshCache.Get(hash);

        if (!mesh)
            mesh = g_application->m_lineMeshCache.Add(hash, points);

        float factor = g_application->m_windowSize.x() / g_application->m_cameraSize.x();
        width *= factor;

        g_application->m_shaderLine.setViewportSize(Vector2{ GL::defaultFramebuffer.viewport().size() })
                                   .setTransformationProjectionMatrix(g_application->m_cameraProjection * g_application->m_globalTransformMatrix)
                                   .setColor(color)
                                   .setWidth(width)
                                   .setSmoothness(width / 2.0f)
                                   .draw(*mesh);
    }

    bool isMousePressed(Mouse mouse)
    {
        return g_application->m_mousePressed[mouse].valueNew && g_application->m_mousePressed[mouse].valueNew != g_application->m_mousePressed[mouse].valueOld;
    }

    bool isMouseReleased(Mouse mouse)
    {
        return !g_application->m_mousePressed[mouse].valueNew && g_application->m_mousePressed[mouse].valueNew != g_application->m_mousePressed[mouse].valueOld;
    }

    bool isMouseDown(Mouse mouse)
    {
        return g_application->m_mousePressed[mouse].valueNew;
    }

    bool isKeyPressed(char key)
    {
        if (!g_application->m_pressedKeys.contains(key))
            return false;

        return g_application->m_pressedKeys[key].valueNew && g_application->m_pressedKeys[key].valueNew != g_application->m_pressedKeys[key].valueOld;
    }

    bool isKeyReleased(char key)
    {
        if (!g_application->m_pressedKeys.contains(key))
            return false;

        return !g_application->m_pressedKeys[key].valueNew && g_application->m_pressedKeys[key].valueNew != g_application->m_pressedKeys[key].valueOld;
    }

    bool isKeyDown(char key)
    {
        return g_application->m_pressedKeys.contains(key) && g_application->m_pressedKeys[key].valueNew;
    }

    float getDeltaTimeMs()
    {
        return g_application->m_frameDeltaMs;
    }

    float getTimeMs()
    {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(now - g_application->m_startApplication).count();
    }

    bool isWindowResized()
    {
        return g_application->m_windowResized;
    }

    UnsignedByte GetAlignment(TextAlign align)
    {
        switch (align)
        {
        case TextAlign::TopLeft: return (UnsignedByte)Text::Alignment::TopLeft;
        case TextAlign::TopMiddle: return (UnsignedByte)Text::Alignment::TopCenter;
        case TextAlign::TopRight: return (UnsignedByte)Text::Alignment::TopRight;
        case TextAlign::MiddleLeft: return (UnsignedByte)Text::Alignment::MiddleLeft;
        case TextAlign::MiddleMiddle: return (UnsignedByte)Text::Alignment::MiddleCenter;
        case TextAlign::MiddleRight: return (UnsignedByte)Text::Alignment::MiddleRight;
        case TextAlign::BottomLeft: return (UnsignedByte)Text::Alignment::LineLeft;
        case TextAlign::BottomMiddle: return (UnsignedByte)Text::Alignment::LineCenter;
        case TextAlign::BottomRight: return (UnsignedByte)Text::Alignment::LineRight;
        }
        return (UnsignedByte)-1;
    }

    void drawText(vec2 position, const std::string& text, float height, col3 color, TextAlign align)
    {
        Magnum::Text::Renderer2D* renderer = g_application->m_textRendererCache.Get(height, GetAlignment(align));
        if (!renderer)
            renderer = g_application->m_textRendererCache.Add(height, GetAlignment(align));

        renderer->render(text);

        g_application->m_textShader.bindVectorTexture(g_application->m_fontGlyphCache->texture())
                                   .setTransformationProjectionMatrix(g_application->m_cameraProjection * CreateTransformation(position, 0.0f, { 1.0f, 1.0f }))
                                   .setColor(color)
                                   .setOutlineRange(0.5f, 1.0f)
                                   .setSmoothness(0.075f)
                                   .draw(renderer->mesh());
    }

    rectangle getTextRectangle(vec2 position, const std::string& text, float height, TextAlign align)
    {
        Magnum::Text::Renderer2D* renderer = g_application->m_textRendererCache.Get(height, GetAlignment(align));
        if (!renderer)
            renderer = g_application->m_textRendererCache.Add(height, GetAlignment(align));

        renderer->render(text);

        auto result = renderer->rectangle();

        return result.translated(position);
    }

    void setTransform(transform t)
    {
        g_application->m_globalTransform = {};
        g_application->m_globalTransformMatrix = CreateTransformation(t.position, t.rotation, {1.0f, 1.0f});
        g_application->m_globalTransform = t;
    }

    transform getTransform()
    {
        return g_application->m_globalTransform;
    }
}

MAGNUM_APPLICATION_MAIN(MyApplication)
