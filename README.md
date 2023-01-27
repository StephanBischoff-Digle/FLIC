# LIC

LIC was implemented by sampling a noise into a texture and encoding the vector field into another texture.
The LIC is then calculated in a shader per fragment.

# FLIC

FLIC is calculated on the CPU only, using one core.
It is very slow, probably due to quite a lot of allocations in the FLIC.cpp:integrate method.

