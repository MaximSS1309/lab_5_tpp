// Variant 22: trampoline jumping simulation (gravity + elastic trampoline surface).
// Structure: handleInput() -> updatePhysics() -> drawScene().

#include <SFML/Graphics.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// ---------------------------------------------------------------- constants
namespace
{
constexpr float kScale     = 80.f;   // pixels per metre
constexpr float kSurfaceY  = 520.f;  // trampoline rest level (pixels)
constexpr float kBallR     = 0.3f;   // body radius, m
constexpr float kBallX     = 420.f;  // body x position (pixels)
constexpr float kTrampL    = 270.f;
constexpr float kTrampR    = 570.f;
constexpr float kFixedStep = 0.001f; // physics sub-step, s
constexpr float kGraphSpan = 6.f;    // seconds shown on the graph
constexpr float kPi        = 3.14159265f;
}

// ------------------------------------------------------------ physical model
// Coordinate h: height of the body's lowest point above the rest level of the
// trampoline (m), positive upwards. h < 0 means contact (mat is deflected).
struct Model
{
    // Parameters
    float mass      = 1.f;      // kg
    float stiffness = 2000.f;   // N/m   (trampoline spring constant k)
    float damping   = 5.f;      // N*s/m (energy loss of the mat)
    float gravity   = 9.81f;    // m/s^2
    float v0        = 0.f;      // initial velocity, m/s (up is positive)
    float h0        = 2.f;      // initial height, m

    // State
    float  h = 2.f;
    float  v = 0.f;
    double time = 0.0;
    int    contacts = 0;
    bool   inContact = false;
    bool   running = false;
    std::vector<float> apexes;  // heights of successive peaks

    void reset()
    {
        h = h0;
        v = v0;
        time = 0.0;
        contacts = 0;
        inContact = false;
        apexes.clear();
    }

    // One step of semi-implicit Euler:
    //   v(n+1) = v(n) + a*dt;  h(n+1) = h(n) + v(n+1)*dt
    void step(float dt)
    {
        const bool contact = h < 0.f;
        float a = -gravity;
        if (contact)
        {
            // Hooke's law + viscous damping; the mat can only push, not pull.
            float force = -stiffness * h - damping * v;
            if (force < 0.f) force = 0.f;
            a += force / mass;
        }

        const float vPrev = v;
        v += a * dt;
        h += v * dt;
        time += dt;

        if (!contact && h < 0.f) ++contacts;
        inContact = h < 0.f;

        // Peak of a flight: velocity changes sign from + to - above the surface.
        if (vPrev > 0.f && v <= 0.f && h >= 0.f)
            apexes.push_back(h);
    }

    float energy() const
    {
        float e = mass * gravity * h + 0.5f * mass * v * v;
        if (h < 0.f) e += 0.5f * stiffness * h * h;
        return e;
    }
};

// --------------------------------------------------------------- application
struct App
{
    Model model;
    double accumulator = 0.0;
    std::deque<std::pair<float, float>> history; // (time, h) for the graph
};

// 1. Input: only changes parameters / run state of the model.
void handleInput(sf::RenderWindow& window, Model& m)
{
    while (const std::optional event = window.pollEvent())
    {
        if (event->is<sf::Event::Closed>())
            window.close();

        if (const auto* key = event->getIf<sf::Event::KeyPressed>())
        {
            using K = sf::Keyboard::Key;
            switch (key->code)
            {
            case K::Space: m.running = !m.running; break;
            case K::R:     m.reset(); m.running = true; break;
            case K::Up:    m.stiffness += 200.f; break;
            case K::Down:  m.stiffness -= 200.f; break;
            case K::Right: m.v0 += 0.5f; break;
            case K::Left:  m.v0 -= 0.5f; break;
            case K::W:     m.mass += 0.5f; break;
            case K::S:     m.mass -= 0.5f; break;
            case K::D:     m.damping += 1.f; break;
            case K::A:     m.damping -= 1.f; break;
            default: break;
            }
            m.stiffness = std::clamp(m.stiffness, 200.f, 20000.f);
            m.v0        = std::clamp(m.v0, -8.f, 8.f);
            m.mass      = std::clamp(m.mass, 0.5f, 5.f);
            m.damping   = std::clamp(m.damping, 0.f, 50.f);

            // Before the first start the new v0 is applied immediately.
            if (m.time == 0.0) m.v = m.v0;
        }
    }
}

// 2. Physics: fixed sub-step integration, independent of frame rate.
void updatePhysics(App& app, float dt)
{
    Model& m = app.model;
    if (!m.running) return;

    app.accumulator += std::min(dt, 0.1f);
    while (app.accumulator >= kFixedStep)
    {
        m.step(kFixedStep);
        app.accumulator -= kFixedStep;
    }

    app.history.emplace_back(static_cast<float>(m.time), m.h);
    while (!app.history.empty() &&
           app.history.front().first < static_cast<float>(m.time) - kGraphSpan)
        app.history.pop_front();
}

// 3. Drawing: only reads the model.
void drawScene(sf::RenderWindow& window, const sf::Font& font, const App& app)
{
    const Model& m = app.model;
    window.clear(sf::Color(20, 20, 30));

    // Ground
    sf::RectangleShape ground({900.f, 60.f});
    ground.setPosition({0.f, 640.f});
    ground.setFillColor(sf::Color(70, 70, 70));
    window.draw(ground);

    // Trampoline legs
    for (float x : {kTrampL, kTrampR})
    {
        sf::RectangleShape leg({6.f, 640.f - kSurfaceY});
        leg.setOrigin({3.f, 0.f});
        leg.setPosition({x, kSurfaceY});
        leg.setFillColor(sf::Color(130, 130, 130));
        window.draw(leg);
    }

    // Trampoline mat: parabola-shaped deflection under the body
    const float sag = std::max(0.f, -m.h) * kScale;
    sf::VertexArray mat(sf::PrimitiveType::TriangleStrip);
    constexpr int N = 40;
    for (int i = 0; i <= N; ++i)
    {
        const float s = static_cast<float>(i) / N;
        const float x = kTrampL + s * (kTrampR - kTrampL);
        const float shape = 1.f - (2.f * s - 1.f) * (2.f * s - 1.f);
        const float y = kSurfaceY + sag * shape;
        mat.append(sf::Vertex{{x, y - 3.f}, sf::Color(60, 160, 255)});
        mat.append(sf::Vertex{{x, y + 3.f}, sf::Color(60, 160, 255)});
    }
    window.draw(mat);

    // Body
    sf::CircleShape ball(kBallR * kScale);
    ball.setOrigin({kBallR * kScale, kBallR * kScale});
    ball.setPosition({kBallX, kSurfaceY - m.h * kScale - kBallR * kScale});
    ball.setFillColor(m.inContact ? sf::Color(255, 140, 0) : sf::Color(255, 220, 50));
    window.draw(ball);

    // Height graph
    const float gx = 600.f, gy = 20.f, gw = 280.f, gh = 170.f;
    const float hMin = -1.f, hMax = 6.f;
    sf::RectangleShape frame({gw, gh});
    frame.setPosition({gx, gy});
    frame.setFillColor(sf::Color(30, 30, 45));
    frame.setOutlineColor(sf::Color(120, 120, 120));
    frame.setOutlineThickness(1.f);
    window.draw(frame);

    const float zeroY = gy + gh - (0.f - hMin) / (hMax - hMin) * gh;
    sf::RectangleShape zeroLine({gw, 1.f});
    zeroLine.setPosition({gx, zeroY});
    zeroLine.setFillColor(sf::Color(90, 90, 90));
    window.draw(zeroLine);

    sf::VertexArray graph(sf::PrimitiveType::LineStrip);
    const float tNow = static_cast<float>(m.time);
    for (const auto& [t, h] : app.history)
    {
        const float x = gx + (t - tNow + kGraphSpan) / kGraphSpan * gw;
        const float hc = std::clamp(h, hMin, hMax);
        const float y = gy + gh - (hc - hMin) / (hMax - hMin) * gh;
        graph.append(sf::Vertex{{x, y}, sf::Color(120, 255, 120)});
    }
    window.draw(graph);

    char buf[512];

    sf::Text graphTitle(font, "h(t), last 6 s  (0 m = trampoline level)", 12);
    graphTitle.setPosition({gx, gy + gh + 4.f});
    graphTitle.setFillColor(sf::Color(180, 180, 180));
    window.draw(graphTitle);

    // Parameters and state
    const float omega = std::sqrt(m.stiffness / m.mass);
    const float contactTime = kPi / omega; // half period of the spring oscillator
    std::snprintf(buf, sizeof(buf),
        "State: %s\n"
        "Time: %.2f s\n"
        "Height h: %.3f m\n"
        "Velocity v: %.3f m/s\n"
        "Contacts: %d\n"
        "Mech. energy: %.2f J\n"
        "Mat deflection: %.3f m\n"
        "\n"
        "PARAMETERS\n"
        "Stiffness k: %.0f N/m   [Up/Down]\n"
        "Initial v0: %.1f m/s   [Left/Right] (up +)\n"
        "Mass m: %.1f kg   [W/S]\n"
        "Damping c: %.0f N*s/m   [A/D]\n"
        "Static sag mg/k: %.3f m\n"
        "Contact time ~ pi*sqrt(m/k): %.3f s",
        m.running ? "RUNNING" : "PAUSED",
        m.time, m.h, m.v, m.contacts, m.energy(),
        std::max(0.f, -m.h),
        m.stiffness, m.v0, m.mass, m.damping,
        m.mass * m.gravity / m.stiffness, contactTime);
    sf::Text info(font, buf, 16);
    info.setPosition({20.f, 20.f});
    info.setFillColor(sf::Color::White);
    window.draw(info);

    // Peaks of the last jumps
    std::string peaks = "PEAK HEIGHTS (last 6), m:\n";
    const std::size_t n = m.apexes.size();
    const std::size_t from = n > 6 ? n - 6 : 0;
    for (std::size_t i = from; i < n; ++i)
    {
        std::snprintf(buf, sizeof(buf), "#%zu: %.3f\n", i + 1, m.apexes[i]);
        peaks += buf;
    }
    sf::Text peakText(font, peaks, 16);
    peakText.setPosition({600.f, 230.f});
    peakText.setFillColor(sf::Color(255, 220, 120));
    window.draw(peakText);

    sf::Text help(font,
        "Space - start / pause\n"
        "R - restart (apply v0)", 16);
    help.setPosition({600.f, 450.f});
    help.setFillColor(sf::Color(160, 200, 255));
    window.draw(help);

    window.display();
}

// ---------------------------------------------------------------------- main
static bool loadFont(sf::Font& font)
{
    const char* paths[] = {
        "../resources/fonts/arial.ttf",
        "resources/fonts/arial.ttf",
        "../../resources/fonts/arial.ttf",
        "C:/Windows/Fonts/arial.ttf",
    };
    for (const char* p : paths)
        if (font.openFromFile(p)) return true;
    return false;
}

int main()
{
    sf::RenderWindow window(sf::VideoMode({900, 700}), "SFML Physics Demo - Trampoline");
    window.setFramerateLimit(60);

    sf::Font font;
    if (!loadFont(font)) return 1;

    App app;
    app.model.reset();

    sf::Clock clock;
    while (window.isOpen())
    {
        const float dt = clock.restart().asSeconds();
        handleInput(window, app.model);
        updatePhysics(app, dt);
        drawScene(window, font, app);
    }
    return 0;
}