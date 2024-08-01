#include "dtkPhysMassSpringSolver.h"

dtk::dtkPhysMassSpringSolver::dtkPhysMassSpringSolver() {
}

dtk::dtkPhysMassSpringSolver::dtkPhysMassSpringSolver(const dtk::dtkPhysMassSpring::Ptr& massSpringSystem) {
    _system = massSpringSystem;
    _time_step = _system->GetTimeStep();

    // current state
    _current_state.resize(3 * _system->GetNumberOfMassPoints());
    for (dtk::dtkID i = 0; i < _system->GetNumberOfMassPoints(); i++) {
        dtk::dtkPhysMassPoint* massPoint = _system->GetMassPoint(i);
        _current_state[3 * i] = massPoint->GetPosition()[0];
        _current_state[3 * i + 1] = massPoint->GetPosition()[1];
        _current_state[3 * i + 2] = massPoint->GetPosition()[2];
    }
    _prev_state = _current_state;
    _spring_directions.resize(3 * _system->GetNumberOfSprings());

    // M 
    TripletList MTriplets;
    _M.resize(3 * _system->GetNumberOfMassPoints(), 3 * _system->GetNumberOfMassPoints());
    for (dtk::dtkID i = 0; i < _system->GetNumberOfMassPoints(); i++) {
        for (dtk::dtkID j = 0; j < 3; j++) {
            MTriplets.push_back(Triplet(3 * i + j, 3 * i + j, _system->GetMassPoint(i)->GetMass()));
        }
    }
    _M.setFromTriplets(MTriplets.begin(), MTriplets.end());

    // L
    TripletList LTriplets;
    _L.resize(3 * _system->GetNumberOfMassPoints(), 3 * _system->GetNumberOfMassPoints());
    for (dtk::dtkID i = 0;i < _system->GetNumberOfSprings();i++) {
        double stiffness = _system->GetSpring(i)->GetStiffness();
        dtk::dtkPhysSpring* spring = _system->GetSpring(i);
        dtk::dtkID id1 = spring->GetFirstVertex()->GetPointID();// TODO GetFirstVertex修改成GetFirstPoint();
        dtk::dtkID id2 = spring->GetSecondVertex()->GetPointID();
        for (dtk::dtkID j = 0; j < 3; j++) {
            LTriplets.push_back(Triplet(3 * id1 + j, 3 * id1 + j, 1 * stiffness));
            LTriplets.push_back(Triplet(3 * id1 + j, 3 * id2 + j, -1 * stiffness));
            LTriplets.push_back(Triplet(3 * id2 + j, 3 * id1 + j, -1 * stiffness));
            LTriplets.push_back(Triplet(3 * id2 + j, 3 * id2 + j, 1 * stiffness));
        }
    }
    _L.setFromTriplets(LTriplets.begin(), LTriplets.end());

    // J
    TripletList JTriplets;
    _J.resize(3 * _system->GetNumberOfMassPoints(), 3 * _system->GetNumberOfSprings());
    for (dtk::dtkID i = 0;i < _system->GetNumberOfSprings();i++) {
        double stiffness = _system->GetSpring(i)->GetStiffness();
        dtk::dtkPhysSpring* spring = _system->GetSpring(i);
        dtk::dtkID id1 = spring->GetFirstVertex()->GetPointID();
        dtk::dtkID id2 = spring->GetSecondVertex()->GetPointID();
        for (unsigned int j = 0; j < 3; j++) {
            JTriplets.push_back(
                Triplet(3 * id1 + j, 3 * i + j, 1 * stiffness));
            JTriplets.push_back(
                Triplet(3 * id2 + j, 3 * i + j, -1 * stiffness));
        }
    }
    _J.setFromTriplets(JTriplets.begin(), JTriplets.end());

    // pre-factor 
    double h2 = _system->GetTimeStep() * _system->GetTimeStep();
    SparseMatrix A = _M + h2 * _L;
    _system_matrix.compute(A);
}

void dtk::dtkPhysMassSpringSolver::solve(unsigned int iter_num) {
    float damping_factor = _system->GetDefaultDamp();

    // update inertial term
    _inertial_term = _M * ((damping_factor + 1) * (_current_state)-damping_factor * _prev_state);
    _prev_state = _current_state;

    // perform steps
    for (unsigned int i = 0; i < iter_num; i++) {
        localStep();
        globalStep();
    }
}

void dtk::dtkPhysMassSpringSolver::localStep() {
    for (dtk::dtkID id = 0;id < _system->GetNumberOfSprings();id++) {
        dtk::dtkPhysSpring* spring = _system->GetSpring(id);
        dtk::dtkID id1 = spring->GetFirstVertex()->GetPointID();
        dtk::dtkID id2 = spring->GetSecondVertex()->GetPointID();
        double rest_length = spring->GetRestLength();
        Vector3f p12(
            _current_state[3 * id1 + 0] - _current_state[3 * id2 + 0],
            _current_state[3 * id1 + 1] - _current_state[3 * id2 + 1],
            _current_state[3 * id1 + 2] - _current_state[3 * id2 + 2]
        );

        p12.normalize();
        _spring_directions[3 * id + 0] = rest_length * p12[0];
        _spring_directions[3 * id + 1] = rest_length * p12[1];
        _spring_directions[3 * id + 2] = rest_length * p12[2];
    }
}

void dtk::dtkPhysMassSpringSolver::globalStep() {
    float h2 = _system->GetTimeStep() * _system->GetTimeStep(); // shorthand

    // compute right hand side
    // dtk::dtkDouble3 fext(0, 0, 0);
    dtk::dtkDouble3 fext = _system->GetDefaultGravityAccel();   // TODO: change to  _system->GetImpulseForce()
    VectorXf fext_force = VectorXf(Vector3f(fext.x, fext.y, fext.z).replicate(_system->GetNumberOfMassPoints(), 1));
    VectorXf b = _inertial_term + h2 * _J * _spring_directions + h2 * fext_force;

    // std::cout << _spring_directions << std::endl;

    // solve system and update state
    _current_state = _system_matrix.solve(b);
}