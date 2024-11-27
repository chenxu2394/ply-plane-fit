from OpenGL.GL import *
from OpenGL.GLUT import *

def show_version():
    print("OpenGL Version:", glGetString(GL_VERSION))

glutInit()
glutInitDisplayMode(GLUT_RGBA)
glutCreateWindow("OpenGL Version")
glutDisplayFunc(show_version)
glutMainLoop()
