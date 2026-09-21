/**
 * @file
 */

#include "micras/sim/view/control_panel.hpp"

#ifdef MICRAS_VIEWER

    #include <algorithm>
    #include <format>
    #include <utility>

    #include <GLFW/glfw3.h>
    #include <imgui.h>
    #include <imgui_impl_glfw.h>
    #include <imgui_impl_opengl3.h>
    #include <implot.h>

namespace micras::sim {
namespace {
/**
 * @brief Range of the speed limiter, in simulated seconds per wall second.
 *
 * @note Zero means no limit, which is what a headless run does.
 */
///@{
constexpr double speed_minimum{0.0};
constexpr double speed_maximum{10.0};
///@}

/**
 * @brief Pool variables the panel plots, in the order they are drawn.
 */
constexpr std::array<const char*, 4> plotted_variables{
    "Desired Linear Speed", "Odometry Linear Velocity", "Desired Angular Speed", "Odometry Angular Velocity"
};

/**
 * @brief Names of the DIP switches, as Interface::DipSwitchPins orders them.
 */
constexpr std::array<const char*, InterfaceInput::dip_switch_count> dip_names{"fan", "diagonal", "boost", "risky"};

/**
 * @brief Name of a Micras::State value.
 *
 * @note The negated comparison is the NaN guard: an unmapped pool variable
 *       reads as NaN, and every comparison with NaN is false.
 *
 * @param state Numeric state as reported by the firmware pool.
 * @return State name, or "?" when out of range or not reported yet.
 */
const char* state_name(double state) {
    static constexpr std::array<const char*, 7> names{"INIT",      "IDLE", "WAIT_FOR_RUN", "RUN", "WAIT_FOR_CALIBRATE",
                                                      "CALIBRATE", "ERROR"};
    const auto                                  index = static_cast<std::size_t>(state);

    if (not(state >= 0.0) or index >= names.size()) {
        return "?";
    }

    return names.at(index);
}

/**
 * @brief Draw a colour swatch for one addressable LED.
 *
 * @param label Label shown beside it.
 * @param colour Colour to show.
 */
void draw_swatch(const std::string& label, const Rgb& colour) {
    const ImVec4 value(colour.red / 255.0F, colour.green / 255.0F, colour.blue / 255.0F, 1.0F);
    ImGui::ColorButton(label.c_str(), value, ImGuiColorEditFlags_NoTooltip, ImVec2(24, 24));
    ImGui::SameLine();
    ImGui::TextUnformatted(label.c_str());
}
}  // namespace

PlotTrace::PlotTrace(std::string variable) : variable{std::move(variable)} {
    this->sample_times.reserve(capacity);
    this->sample_values.reserve(capacity);
}

void PlotTrace::sample(double time, const Telemetry& telemetry) {
    if (this->sample_times.size() == capacity) {
        this->sample_times.erase(this->sample_times.begin());
        this->sample_values.erase(this->sample_values.begin());
    }

    this->sample_times.push_back(time);
    this->sample_values.push_back(telemetry.value_of(this->variable));
}

ControlPanel::ControlPanel(GLFWwindow* window, ProxyState& state, const Telemetry& telemetry) :
    state{state}, telemetry{telemetry} {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    for (const char* variable : plotted_variables) {
        this->traces.emplace_back(variable);
    }
}

ControlPanel::~ControlPanel() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
}

void ControlPanel::take_over() {
    this->touched = true;
    this->state.interface_input.driven_by_human = true;
}

void ControlPanel::sample(double time) {
    for (PlotTrace& trace : this->traces) {
        trace.sample(time, this->telemetry);
    }
}

PanelRequest ControlPanel::draw(const Simulation& simulation, bool paused) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360, 620), ImGuiCond_FirstUseEver);
    ImGui::Begin("micras");

    ImGui::TextUnformatted(std::format("tick {}", simulation.tick()).c_str());
    ImGui::TextUnformatted(std::format("state {}", state_name(this->telemetry.value_of("FSM State"))).c_str());
    ImGui::Separator();

    this->request.paused_changed = false;
    this->draw_board(paused);
    ImGui::Separator();
    this->draw_plots();

    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    return this->request;
}

void ControlPanel::draw_board(bool paused) {
    InterfaceInput& input = this->state.interface_input;

    ImGui::TextUnformatted("run");
    bool wanted = paused;

    if (ImGui::Checkbox("paused", &wanted)) {
        this->request.paused = wanted;
        this->request.paused_changed = true;
    }

    ImGui::SameLine();

    if (ImGui::Button("step")) {
        this->request.steps++;
    }

    ImGui::SameLine();

    if (ImGui::Button("quit")) {
        this->request.quit = true;
    }

    ImGui::SliderScalar("speed", ImGuiDataType_Double, &this->request.speed, &speed_minimum, &speed_maximum, "%.1f x");

    ImGui::Spacing();
    ImGui::TextUnformatted("board");

    ImGui::Button("button");

    if (ImGui::IsItemActivated()) {
        this->take_over();
        input.button_pressed = true;
    } else if (ImGui::IsItemDeactivated()) {
        input.button_pressed = false;
    }

    for (std::size_t i = 0; i < dip_names.size(); i++) {
        bool switched = input.dip_switches.at(i);

        if (ImGui::Checkbox(dip_names.at(i), &switched)) {
            this->take_over();
            input.dip_switches.at(i) = switched;
        }

        if (i + 1 < dip_names.size()) {
            ImGui::SameLine();
        }
    }

    if (ImGui::Checkbox("fan allowed", &this->state.overrides.fan_enabled)) {
        this->take_over();
    }

    ImGui::Spacing();
    ImGui::TextUnformatted("shown");

    const InterfaceOutput& output = this->state.interface_output;
    draw_swatch("led", output.led_on ? Rgb{255, 255, 255} : Rgb{40, 40, 40});

    for (std::size_t i = 0; i < output.argb.size(); i++) {
        draw_swatch(std::format("argb {}", i), output.argb.at(i));
    }

    ImGui::TextUnformatted(std::format("buzzer {} Hz", output.buzzer_frequency).c_str());

    ImGui::Spacing();
    ImGui::TextUnformatted("driven");

    const Actuators& actuators = this->state.actuators;
    ImGui::TextUnformatted(
        std::format("wheels {:6.1f} {:6.1f}", actuators.left_command, actuators.right_command).c_str()
    );
    ImGui::TextUnformatted(std::format("fan {:6.1f}", actuators.fan_speed).c_str());

    const Sensors& sensors = this->state.sensors;
    ImGui::TextUnformatted(std::format(
                               "walls {:5.3f} {:5.3f} {:5.3f} {:5.3f}", sensors.wall_adc_readings.at(0),
                               sensors.wall_adc_readings.at(1), sensors.wall_adc_readings.at(2),
                               sensors.wall_adc_readings.at(3)
    )
                               .c_str());
}

void ControlPanel::draw_plots() {
    for (const PlotTrace& trace : this->traces) {
        if (not ImPlot::BeginPlot(trace.name().c_str(), ImVec2(-1, 110))) {
            continue;
        }

        ImPlot::SetupAxes("s", nullptr, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);

        if (not trace.times().empty()) {
            ImPlot::PlotLine(
                trace.name().c_str(), trace.times().data(), trace.values().data(),
                static_cast<int>(trace.times().size())
            );
        }

        ImPlot::EndPlot();
    }
}
}  // namespace micras::sim

#else  // MICRAS_VIEWER

namespace micras::sim {
PlotTrace::PlotTrace(std::string variable) : variable{std::move(variable)} { }

void PlotTrace::sample(double /*time*/, const Telemetry& /*telemetry*/) { }

ControlPanel::ControlPanel(GLFWwindow* /*window*/, ProxyState& state, const Telemetry& telemetry) :
    state{state}, telemetry{telemetry} { }

ControlPanel::~ControlPanel() = default;

void ControlPanel::sample(double /*time*/) { }

PanelRequest ControlPanel::draw(const Simulation& /*simulation*/, bool /*paused*/) {
    return this->request;
}

void ControlPanel::draw_board(bool /*paused*/) { }

void ControlPanel::draw_plots() { }

void ControlPanel::take_over() { }
}  // namespace micras::sim

#endif  // MICRAS_VIEWER
