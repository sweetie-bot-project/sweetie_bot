//#version 300 es

//uniform mediump vec4 color;
precision mediump float; // for raspberry
varying mediump vec4 texc;
uniform sampler2D texture;

void main()
{
    vec4 my_color;
   my_color=texture2D(texture, texc.st);
  // if (my_color.a!=1) discard;
  // my_color.a=0.5;
   //TODO отбрасывать точки с алфа=0
   gl_FragColor =my_color;
}
