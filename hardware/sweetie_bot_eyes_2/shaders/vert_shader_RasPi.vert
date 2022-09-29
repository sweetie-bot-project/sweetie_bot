//#version 300 es

attribute highp vec2 position;
attribute highp vec4 tex_cord;
uniform highp mat4 rot_matrix;
uniform highp mat4 trans_matrix;
precision mediump float; // for raspberry
varying mediump vec4 texc;

void main(void)
{
   // gl_Position = vec4(position.x, position.y, position.z, 1.0);
    vec4 my_ver;
    my_ver=vec4(position.x, position.y, 0, 1.0);
   gl_Position = rot_matrix *trans_matrix* my_ver;
    //gl_Position = my_ver;
    //texc=tex_matrix*my_ver;
    texc=tex_cord;
}
