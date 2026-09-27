/**
 * @file
 *
 * @brief What the window and the recorder set up alike: the model camera they start from and what
 * they draw.
 */

#ifndef MICRAS_SIM_VIEW_CAMERA_HPP
#define MICRAS_SIM_VIEW_CAMERA_HPP

#include <algorithm>
#include <string>

#include <mujoco/mujoco.h>

namespace micras::sim {
/**
 * @brief Point a camera at one of the model's cameras.
 *
 * @param model Model to look the camera up in.
 * @param name Name of the camera.
 * @param camera Camera to point.
 * @param error Filled with the cameras the model has when it has none by that name.
 * @return False when the name was rejected.
 */
inline bool select_model_camera(const mjModel* model, const std::string& name, mjvCamera& camera, std::string& error) {
    const int id = mj_name2id(model, mjOBJ_CAMERA, name.c_str());

    if (id < 0) {
        error = "model has no camera named '" + name + "'; it has";

        for (int i = 0; i < model->ncam; i++) {
            const char* found = mj_id2name(model, mjOBJ_CAMERA, i);
            error += std::string(" '") + (found == nullptr ? "?" : found) + "'";
        }

        error += " and 'free'";
        return false;
    }

    camera.type = mjCAMERA_FIXED;
    camera.fixedcamid = id;
    return true;
}

/**
 * @brief Draw every geom group, and the rays of the range sensors, which are usually the
 * interesting part of a run.
 *
 * @param option Options to set.
 */
inline void show_everything(mjvOption& option) {
    option.flags[mjVIS_RANGEFINDER] = 1;
    std::ranges::fill(option.geomgroup, 1);
}
}  // namespace micras::sim

#endif  // MICRAS_SIM_VIEW_CAMERA_HPP
