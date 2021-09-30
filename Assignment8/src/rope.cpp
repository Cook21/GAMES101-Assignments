#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

const float kd = 0.00005;
    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
    Vector2D seg = (end - start) / (num_nodes - 1);
    masses.reserve(num_nodes);
    springs.reserve(num_nodes - 1);
    for (int i = 0; i < num_nodes; i++) {
        masses.push_back(new Mass(start + i * seg, node_mass, false));
        if (i > 0) {
            springs.push_back(new Spring(masses[i - 1], masses[i], k));
        }
    }
    //Comment-in this part when you implement the constructor
    //pinned_nodes存储被固定节点的下标
    for (auto& i : pinned_nodes) {
        masses[i]->pinned = true;
    }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
    for (auto& s : springs) {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
        auto direction = s->m2->position - s->m1->position;
        auto distance = direction.norm();
        Vector2D force = s->k * direction / distance * (distance - s->rest_length);
        s->m1->forces += force;
        s->m2->forces -= force;
        }

    for (auto& m : masses) {
        if (!m->pinned) {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
            m->forces += 9.8 * m->mass * gravity;
            m->last_position = m->position;
            m->velocity += m->forces / m->mass * delta_t;
            m->position = m->last_position + m->velocity * delta_t;
                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
    for (auto& s : springs) {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
        }

    for (auto& m : masses) {
        if (!m->pinned) {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                
                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
}
