import sys
import os

# Укажите путь к библиотекам FreeCAD
freecad_path = r"D:\Program Files\FreeCAD 0.21\bin"  
sys.path.append(freecad_path)

# Импортируйте нужные модули
import FreeCAD
import FreeCADGui  

# Получаем неровную триангулярную поверхность
surface = FreeCAD.ActiveDocument.Objects["mesh"]

# Извлекаем вершины поверхности
vertices = surface.Shape.Vertexes

# Сортируем вершины по высоте по оси Z
vertices.sort(key=lambda vertex: vertex.Z)

# Ищем точки пика, где поверхность начинает спускаться
peak_vertices = []
for i in range(1, len(vertices) - 1):
    if vertices[i].Z < vertices[i - 1].Z and vertices[i].Z < vertices[i + 1].Z:
        peak_vertices.append(vertices[i])

# Создаем список выпуклых поверхностей
convex_surfaces = []
for peak_vertex in peak_vertices:
    # Выбираем грани, примыкающие к точке пика
    adjacent_faces = surface.Shape.FacesFromNode(peak_vertex)

    # Создаем новую выпуклую оболочку из выбранных граней
    convex_hull = FreeCAD.FemTools.makeConvexHull([face.Shape for face in adjacent_faces])

    # Создаем новую выпуклую поверхность из выпуклой оболочки
    convex_surface = FreeCAD.FemTools.makeSurfaceFromShape(convex_hull, ConvexSurface)

    convex_surfaces.append(convex_surface)

# Добавляем выпуклые поверхности в активный документ
for convex_surface in convex_surfaces:
    FreeCAD.ActiveDocument.addObject(PartFeature, convex_surface)