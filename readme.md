# BVH Parser for Python
Project to parse BVH files with:
- python importer
- structure management (joints and channel classes)
- complete kinematic chains, including rototranslations
- complementary functions to efficiently manage relative and absolute positioning
- euler angles

## Important
- Some bvh files use different namings for joints. In particular, the root joint must be "Hip" to get this module working. We'll make it more general in the future.
- Feel free to contribute and push :)

## Developers
- Marco Milesi for coding (integrations, mathematics, optimizations and scalability) 
- Gianbattista Madaschi for mathematics

## Credits
- Developed for reasearch purposes in University of Bergamo (Italy)
- Based on https://github.com/20tab/bvh-python
