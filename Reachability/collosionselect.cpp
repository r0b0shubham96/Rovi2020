 
       if (from_side){
    // Finds solutions from side of object
        for (double yaw = 0; yaw < 2*rw::math::Pi; yaw += rw::math::Deg2Rad*1) {
            rw::math::RPY<> rotTarget_side(0, yaw, 0);
            // Rotates axis so following grasp target from the side
            rw::math::Vector3D<> posTarget = object_frame->getTransform(state).P(); // Get position from object and uses it for targetframe
            rw::math::Transform3D<> newTarget (posTarget, rotyaw90.toRotation3D()*rotroll90.toRotation3D()*rotTarget_side.toRotation3D());
            target->moveTo(newTarget, state);
            std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", robot, workcell, state);
           
        }
    }
       if(!from_top){
        // Finds solutions from top of object
        for (double roll = 0; roll < 2*rw::math::Pi; roll += rw::math::Deg2Rad*1) {
            // Rotates axis so following grasp target from the top
            rw::math::RPY<> rotTarget_up(roll, 0, 0); // set to zero if not skew grapping
            rw::math::Vector3D<> posTarget = object_frame->getTransform(state).P();
    
            rw::math::Transform3D<> newTarget (posTarget, object_frame->getTransform(state).R()*rotTarget_up.toRotation3D());
            target->moveTo(newTarget, state);
            std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", robot, workcell, state);
}
           
