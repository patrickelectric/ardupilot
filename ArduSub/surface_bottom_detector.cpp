// Jacob Walser: jacob@bluerobotics.com

#include "Sub.h"

// counter to verify contact with bottom
static uint32_t bottom_detector_count = 0;
static uint32_t surface_detector_count = 0;
static float current_depth = 0;

// checks if we have have hit bottom or surface and updates the ap.at_bottom and ap.at_surface flags
// called at MAIN_LOOP_RATE
// ToDo: doesn't need to be called this fast
void Sub::update_surface_and_bottom_detector()
{
    if (!motors.armed()) { // only update when armed
        set_surfaced(false);
        set_bottomed(false);
        return;
    }

    Vector3f velocity;
    ahrs.get_velocity_NED(velocity);

    // check that we are not moving up or down
    bool vel_stationary = velocity.z > -0.05 && velocity.z < 0.05;

    static float alt_control_error = pos_control.get_alt_error();
    static int maybe_bottom = 0;

    if(pos_control.get_alt_error() < 0 && (pos_control.get_alt_error() - alt_control_error) > 0) {
        maybe_bottom += 1;
    } else if(pos_control.get_alt_error() < 0) {
        alt_control_error = pos_control.get_alt_error();
    } else {
        maybe_bottom = 0;
    }

    gcs_send_text_fmt(MAV_SEVERITY_INFO, "> %d | %d\n", alt_control_error < 0, (pos_control.get_alt_error() - alt_control_error) > 0);
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "vs:%d|vz:%.2f|alt_e:%.2f|altce:%.2f|mb %d\n", vel_stationary, velocity.z, pos_control.get_alt_error(), alt_control_error, maybe_bottom);
    // TODO: REmove
    current_depth = barometer.get_altitude(); // cm
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "alt_set: %.3f \t alt: %.3f\n", pos_control.get_alt_target(), inertial_nav.get_altitude());
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "cd: %.3f\n", current_depth);

    //printf("%d | %f \t %f \t %f\n", vel_stationary, velocity.z, pos_control.get_alt_error(), pos_control.get_alt_target());

    const float surface_depth = g.surface_depth.get()/100.0f;
    if (ap.depth_sensor_present && sensor_health.depth) { // we can use the external pressure sensor for a very accurate and current measure of our z axis position
        current_depth = barometer.get_altitude(); // cm


        if (ap.at_surface) {
            set_surfaced(current_depth > surface_depth - 0.05); // add a 5cm buffer so it doesn't trigger too often
        } else {
            set_surfaced(current_depth > surface_depth); // If we are above surface depth, we are surfaced
        }
        // Vehicle can't be in surface and bottom at the same time
        if(ap.at_surface) {
            return;
        }


        if ((motors.limit.throttle_lower && vel_stationary) || maybe_bottom > 2) {
            // bottom criteria met - increment the counter and check if we've triggered
            set_bottomed(true);

        } else {
            set_bottomed(false);
        }

        // with no external baro, the only thing we have to go by is a vertical velocity estimate
    } else if (vel_stationary) {
        if (motors.limit.throttle_upper) {

            // surface criteria met, increment counter and see if we've triggered
            set_surfaced(true);

        } else if (motors.limit.throttle_lower) {
            // bottom criteria met, increment counter and see if we've triggered
            set_bottomed(true);

        } else { // we're not at the limits of throttle, so reset both detectors
            set_surfaced(false);
            set_bottomed(false);
        }

    } else { // we're moving up or down, so reset both detectors
        set_surfaced(false);
        set_bottomed(false);
    }
}

void Sub::set_surfaced(bool at_surface)
{


    if (ap.at_surface == at_surface) { // do nothing if state unchanged
        return;
    }

    ap.at_surface = at_surface;

    surface_detector_count = 0;

    if (ap.at_surface) {
        Log_Write_Event(DATA_SURFACED);
        gcs_send_text(MAV_SEVERITY_WARNING, "Vehicle in surface.");
    } else {
        Log_Write_Event(DATA_NOT_SURFACED);
    }
}

void Sub::set_bottomed(bool at_bottom)
{

    if (ap.at_bottom == at_bottom) { // do nothing if state unchanged
        return;
    }

    ap.at_bottom = at_bottom;

    bottom_detector_count = 0;

    if (ap.at_bottom) {
        Log_Write_Event(DATA_BOTTOMED);
        gcs_send_text(MAV_SEVERITY_WARNING, "Vehicle in bottom.");
    } else {
        Log_Write_Event(DATA_NOT_BOTTOMED);
    }
}
