void main()
{
    gl_Position = ftransform();
    gl_TexCoord[0].xy = 0.5 * (gl_Position.xy / gl_Position.w + 1.0);
}










