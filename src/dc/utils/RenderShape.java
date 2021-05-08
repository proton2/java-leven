package dc.utils;

public enum RenderShape {
    RenderShape_Cube,
    RenderShape_WireCube,
    RenderShape_Sphere,
    RenderShape_Line,
    RenderShape_SIZE,
    RenderShape_CubeArrayCoords,

    // None is placed after size to prevent it being picked as a valid value
    RenderShape_None,
}
