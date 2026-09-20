#!/usr/bin/env python3
import argparse
import pinocchio as pin


parser = argparse.ArgumentParser()
parser.add_argument('urdf')
parser.add_argument('indices', nargs='+', type=int)
args = parser.parse_args()
model = pin.buildModelFromUrdf(args.urdf)
geometry = pin.buildGeomFromUrdf(model, args.urdf, pin.GeometryType.COLLISION)
geometry.addAllCollisionPairs()
for index in args.indices:
    pair = geometry.collisionPairs[index]
    print(index, geometry.geometryObjects[pair.first].name,
          geometry.geometryObjects[pair.second].name)
