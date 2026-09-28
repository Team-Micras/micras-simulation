/**
 * @file
 */

#include "micras/sim/view/control_panel.hpp"

#ifdef MICRAS_SIM_VIEWER

    #include <algorithm>
    #include <cstddef>
    #include <format>
    #include <limits>
    #include <string>
    #include <utility>

    #include <GLFW/glfw3.h>
    #include <imgui.h>
    #include <imgui_impl_glfw.h>
    #include <imgui_impl_opengl3.h>
    #include <implot.h>

    #include "micras/sim/core/simulation.hpp"
    #include "micras/sim/core/variable_source.hpp"
    #include "micras/sim/view/panel_spec.hpp"

namespace micras::sim {
PlotTrace::PlotTrace(std::string variable) : variable{std::move(variable)} { }

void PlotTrace::sample(double time, const VariableSource* variables) {
    this->sample_times.at(this->next) = time;
    this->sample_values.at(this->next) =
        variables == nullptr ? std::numeric_limits<double>::quiet_NaN() : variables->value_of(this->variable);
    this->next = (this->next + 1) % capacity;
    this->count = std::min(this->count + 1, capacity);
}

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
 * @brief Switches drawn on one row before the next row starts.
 */
constexpr std::size_t switches_per_row{4};
}  // namespace

/**
 * @brief Draw a colour swatch for one lamp.
 *
 * @param label Label shown beside it.
 * @param colour Colour to show.
 */
static void draw_swatch(const std::string& label, const Colour& colour) {
    const ImVec4 value(colour.red / 255.0F, colour.green / 255.0F, colour.blue / 255.0F, 1.0F);
    ImGui::ColorButton(label.c_str(), value, ImGuiColorEditFlags_NoTooltip, ImVec2(24, 24));
    ImGui::SameLine();
    ImGui::TextUnformatted(label.c_str());
}

ControlPanel::ControlPanel(GLFWwindow* window, std::string title, PanelSpec spec, const VariableSource* variables) :
    title{std::move(title)}, spec{std::move(spec)}, variables{variables} {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    for (const std::string& variable : this->spec.plots) {
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
    if (not this->touched and this->spec.take_over) {
        this->spec.take_over();
    }

    this->touched = true;
}

void ControlPanel::sample(double time) {
    for (PlotTrace& trace : this->traces) {
        trace.sample(time, this->variables);
    }
}

PanelRequest ControlPanel::draw(const Simulation& simulation, bool paused) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360, 620), ImGuiCond_FirstUseEver);
    ImGui::Begin(this->title.c_str());

    ImGui::TextUnformatted(std::format("tick {}", simulation.tick()).c_str());

    if (this->spec.state.has_value()) {
        const double state = this->variables == nullptr ? std::numeric_limits<double>::quiet_NaN() :
                                                          this->variables->value_of(this->spec.state->variable);
        ImGui::TextUnformatted(std::format("state {}", this->spec.state->name_of(state)).c_str());
    }

    ImGui::Separator();

    this->request.paused_changed = false;
    this->request.step = false;
    this->draw_board(paused);
    ImGui::Separator();
    this->draw_plots();

    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    return this->request;
}

void ControlPanel::draw_board(bool paused) {
    ImGui::TextUnformatted("run");
    bool wanted = paused;

    if (ImGui::Checkbox("paused", &wanted)) {
        this->request.paused = wanted;
        this->request.paused_changed = true;
    }

    ImGui::SameLine();

    if (ImGui::Button("step")) {
        this->request.step = true;
    }

    ImGui::SameLine();

    if (ImGui::Button("quit")) {
        this->request.quit = true;
    }

    ImGui::SliderScalar("speed", ImGuiDataType_Double, &this->request.speed, &speed_minimum, &speed_maximum, "%.1f x");

    ImGui::Spacing();
    ImGui::TextUnformatted("board");

    for (const PanelButton& button : this->spec.buttons) {
        ImGui::Button(button.name.c_str());

        if (ImGui::IsItemActivated()) {
            this->take_over();
            button.press(true);
        } else if (ImGui::IsItemDeactivated()) {
            button.press(false);
        }
    }

    for (std::size_t i = 0; i < this->spec.switches.size(); i++) {
        const PanelSwitch& panel_switch = this->spec.switches.at(i);
        bool               switched = panel_switch.state();

        if (i % switches_per_row != 0) {
            ImGui::SameLine();
        }

        if (ImGui::Checkbox(panel_switch.name.c_str(), &switched)) {
            this->take_over();
            panel_switch.set(switched);
        }
    }

    if (not this->spec.lamps.empty()) {
        ImGui::Spacing();
        ImGui::TextUnformatted("shown");
    }

    for (const PanelLamp& lamp : this->spec.lamps) {
        draw_swatch(lamp.name, lamp.colour());
    }

    if (not this->spec.readouts.empty()) {
        ImGui::Spacing();
        ImGui::TextUnformatted("driven");
    }

    for (const PanelReadout& readout : this->spec.readouts) {
        ImGui::TextUnformatted(std::format("{} {}", readout.name, readout.text()).c_str());
    }
}

void ControlPanel::draw_plots() {
    for (const PlotTrace& trace : this->traces) {
        if (not ImPlot::BeginPlot(trace.name().c_str(), ImVec2(-1, 110))) {
            continue;
        }

        ImPlot::SetupAxes("s", nullptr, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);

        if (trace.size() > 0) {
            ImPlot::PlotLine(
                trace.name().c_str(), trace.times().data(), trace.values().data(), static_cast<int>(trace.size()), 0,
                static_cast<int>(trace.oldest())
            );
        }

        ImPlot::EndPlot();
    }
}
}  // namespace micras::sim

#else  // MICRAS_SIM_VIEWER

namespace micras::sim {
ControlPanel::~ControlPanel() = default;
}  // namespace micras::sim

#endif  // MICRAS_SIM_VIEWER
