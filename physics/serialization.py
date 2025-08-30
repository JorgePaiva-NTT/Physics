
import json

from .Particle import Particle
from .Vec2 import Vec2
from .DistanceConstraint import DistanceConstraint
from .spring import Spring
from .world import World

class PhysicsEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, Vec2):
            return {'__class__': 'Vec2', 'x': obj.x, 'y': obj.y}
        if isinstance(obj, Particle):
            return {
                '__class__': 'Particle',
                'pos': obj.pos,
                'old_pos': obj.old_pos,
                'acceleration': obj.acceleration,
                'radius': obj.radius,
                'mass': obj.mass,
                'fixed': obj.fixed
            }
        if isinstance(obj, DistanceConstraint):
            return {'__class__': 'DistanceConstraint', 'p1_idx': obj.p1_idx, 'p2_idx': obj.p2_idx, 'distance': obj.distance}
        if isinstance(obj, Spring):
            return {'__class__': 'Spring', 'p1_idx': obj.p1_idx, 'p2_idx': obj.p2_idx, 'length': obj.length, 'stiffness': obj.stiffness}
        return super().default(obj)

def physics_decoder(dct):
    if '__class__' in dct:
        class_name = dct['__class__']
        if class_name == 'Vec2':
            return Vec2(dct['x'], dct['y'])
        if class_name == 'Particle':
            p = Particle(dct['pos'], dct['radius'], dct['mass'], dct['fixed'])
            p.old_pos = dct['old_pos']
            p.acceleration = dct['acceleration']
            return p
        if class_name == 'DistanceConstraint':
            # We will resolve particle indices after all particles are decoded
            return {'__class__': 'DistanceConstraint', 'p1_idx': dct['p1_idx'], 'p2_idx': dct['p2_idx'], 'distance': dct['distance']}
        if class_name == 'Spring':
            # We will resolve particle indices after all particles are decoded
            return {'__class__': 'Spring', 'p1_idx': dct['p1_idx'], 'p2_idx': dct['p2_idx'], 'length': dct['length'], 'stiffness': dct['stiffness']}
    return dct

def save_world(world, filename):
    data = {
        'width': world.width,
        'height': world.height,
        'gravity': world.gravity,
        'particles': world.particles,
        'constraints': []
    }
    # Store particle indices for constraints
    particle_to_idx = {p: i for i, p in enumerate(world.particles)}
    for c in world.constraints:
        if isinstance(c, DistanceConstraint):
            data['constraints'].append({
                '__class__': 'DistanceConstraint',
                'p1_idx': particle_to_idx[c.p1],
                'p2_idx': particle_to_idx[c.p2],
                'distance': c.distance
            })
        elif isinstance(c, Spring):
            data['constraints'].append({
                '__class__': 'Spring',
                'p1_idx': particle_to_idx[c.p1],
                'p2_idx': particle_to_idx[c.p2],
                'length': c.length,
                'stiffness': c.stiffness
            })

    with open(filename, 'w') as f:
        json.dump(data, f, cls=PhysicsEncoder, indent=4)

def load_world(filename):
    with open(filename, 'r') as f:
        data = json.load(f, object_hook=physics_decoder)

    world = World(data['width'], data['height'], data['gravity'])
    world.particles = data['particles']

    # Resolve particle indices for constraints
    for c_data in data['constraints']:
        class_name = c_data['__class__']
        p1 = world.particles[c_data['p1_idx']]
        p2 = world.particles[c_data['p2_idx']]
        if class_name == 'DistanceConstraint':
            world.add_constraint(DistanceConstraint(p1, p2, c_data['distance']))
        elif class_name == 'Spring':
            world.add_constraint(Spring(p1, p2, c_data['length'], c_data['stiffness']))
    return world
