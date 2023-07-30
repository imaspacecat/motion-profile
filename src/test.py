        m_direction = shouldFlipAcceleration(initial, goal) ? -1 : 1;
        m_constraints = constraints;
        m_initial = direct(initial);
        m_goal = direct(goal);

        if (m_initial.velocity > m_constraints.maxVelocity) {
            m_initial.velocity = m_constraints.maxVelocity;
        }

        // Deal with a possibly truncated motion profile (with nonzero initial or
        // final velocity) by calculating the parameters as if the profile began and
        // ended at zero velocity
        double initial_vel_t = m_initial.velocity / m_constraints.maxAcceleration;
        double initial_vel_d = initial_vel_t * initial_vel_t * m_constraints.maxAcceleration / 2.0;

        double final_vel_t = m_goal.velocity / m_constraints.maxAcceleration;
        double final_vel_d = final_vel_t * final_vel_t * m_constraints.maxAcceleration / 2.0;

        // Now we can calculate the parameters as if it was a full trapezoid instead
        // of a truncated one

        double overall_distance = initial_vel_d + (m_goal.position - m_initial.position)
                + final_vel_d;

        double accel_t = m_constraints.maxVelocity / m_constraints.maxAcceleration;

        double cruise_d = overall_distance - accel_t * accel_t
                * m_constraints.maxAcceleration;

        // Handle the case where the profile never reaches full speed
        if (cruise_d < 0) {
            accel_t = Math.sqrt(overall_distance / m_constraints.maxAcceleration);
            cruise_d = 0;
        }

        m_endAccel = accel_t - initial_vel_t;
        m_endFullSpeed = m_endAccel + cruise_d / m_constraints.maxVelocity;
        m_endDeccel = m_endFullSpeed + accel_t - final_vel_t;