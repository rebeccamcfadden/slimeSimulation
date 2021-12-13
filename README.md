# CSCE 489 - A6 - Rebecca McFadden
### Minecraft Slime Block - Mass Spring Simulation - Rebecca McFadden
-> see [Report](reports/README.md)

#### Creating the Slime

I used trilinear interpolation to fill in particles of even distribution throughout the cube

Todo list:  
- [X] Change A5 to support cube of springs
    - [X] Redo posBuf and norBuf (only render outer points?)
    - [X] Setup structural springs
    - [X] Setup shear springs
    - [X] Setup bending springs
- [X] Jumping - apply force to bottom nodes on key press
- [X] Slime rendering
    - [X] choose outer points to form cube mesh
    - [X] calculate per face normals
    - [X] transparency
    - [X] choose inner points to form inner cube mesh
    - [X] calculate per vertex normals
    - [X] texture coords
- [ ] Extras
    - [X] Test collisions
    - [ ] slime particles?
    - [ ] Spawn a ball at mouse click location
    - [X] User controlled motion
    - [ ] collisions with slime(s)
