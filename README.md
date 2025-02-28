The program displays a render of a mountain and an animated waterfall. 
![image](https://github.com/user-attachments/assets/6cfa63fa-460e-4980-a1ed-00ce2ad7abcf)

__________

**The mountain** has been procedurally generated, following these steps:
1. A flat 2D mesh is made
2. The height of the points is adjusted in such a way as to create the cavity in the middle, using this formula:<br />
     $h = h/2 *h_{max}$ if  $d < 1$<br />
     $h = (1 - (2-d)^2/2) * h_{max}$ otherwise

3. Perlin noise is applied to the height, giving the mesh a more "mountain-y" look.
4. A Bezier curve with 4 control points is applied on the mesh, making room for the waterfall that will be there.

___________

**The waterfall** follows the same Bezier curve as mentioned above, and is represented through particle effects that follow that path.

__________

**The lighting** is made by using deffered rendering, and there are multiple light sources(the snowflakes) moving around in the scene, lighting certain areas.

__________
The background image is a skybox constructed in the Cubemap shader.
