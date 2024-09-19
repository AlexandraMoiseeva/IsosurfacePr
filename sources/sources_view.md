
# 1 -> Сбор литературы на тему склейки поверхностей; #

Самое подходящее на эту тему:
[Статья](./LanWuP14-Stitching.pdf)
В FreeCad то же [Ссылка на руководство](https://docviewer.yandex.ru/view/1130000060644581/?page=2&*=H0%2FZkY21Dfi6PP3MkaIaY3FTkBp7InVybCI6Imh0dHBzOi8vZm9ydW0uZnJlZWNhZC5vcmcvZG93bmxvYWQvZmlsZS5waHA%2FaWQ9MTk1MTE0IiwidGl0bGUiOiJmaWxlLnBocD9pZD0xOTUxMTQiLCJub2lmcmFtZSI6dHJ1ZSwidWlkIjoiMTEzMDAwMDA2MDY0NDU4MSIsInRzIjoxNzI2NTMzODk3MDEzLCJ5dSI6IjM4NTg1NTk5MjE3MjUzOTE3NTYiLCJzZXJwUGFyYW1zIjoidG09MTcyNjUzMzg5MCZ0bGQ9cnUmbGFuZz1lbiZuYW1lPWZpbGUucGhwP2lkPTE5NTExNCZ0ZXh0PWZyZWVjYWQrZzErYmxlbmRpbmcrY29udGludWVseSZ1cmw9aHR0cHMlM0EvL2ZvcnVtLmZyZWVjYWQub3JnL2Rvd25sb2FkL2ZpbGUucGhwJTNGaWQlM0QxOTUxMTQmbHI9MjA1NzEmbWltZT1wZGYmbDEwbj1ydSZzaWduPTAyMzc3YmFkYjM1ZDNiNzhhNWI4MWNjNzA4Zjk0YjE5JmtleW5vPTAifQ%3D%3D&lang=en). 

<img src="Снимок экрана 2024-09-17 054013.png" alt="image" width="400"/>, - здесь я просто соединила соответствующие ребра поверхностей и скруглила Fillet программка лежит в sources - surfaces_join 

<img src="Снимок экрана 2024-09-17 111900.png" alt="image" width="400"/>, - здесь с помощью ранее упомянутого blend surface. Но, к слову, когда поверхности пересекаются, blend surface не так гладка, так что в теории можно дополнить.

<img src="Снимок экрана 2024-09-17 122511.png" alt="image" width="400"/>


[Код на blend surface](https://github.com/tomate44/CurvesWB/blob/master/freecad/Curves/blendSurface.py)
#1 -> Сбор литературы на тему сглаживания поверхностей;#
Тут информации достаточно
Все в целом расписано здесь [Статья](./smoothing.pdf), еще один с алгоритмом - [Статья](./s00158-021-03027-6.pdf). 
А так, кажется, используются, либо метод Таубина, либо Лапласа. Все есть в FreeCad'е, также можно импортировать через библиотеку pyvista [PyVista surface smoothing](https://docs.pyvista.org/examples/01-filter/surface-smoothing.html). 

[А тут напрямую код из репозитория FreeCad](https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/Mesh/App/Core/Smoothing.cpp)
# 2-3 -> Экспорт поверхности из тепловой задачи через АПИ; #
Все осуществляет export1.py в export_data - На данный момент выгружает поверхности на втором шаге с 0 до 1 с шагом 0.1, 10 штук. А также саму сетку. Поверхности в папке data/isosurface_folder, сама сетка - mesh.stl в папке data
# 2-3 -> Импорт полученных поверхностей в куформ (для визуального анализа); #
Скрипт export2.py, но нужно иметь активный процесс. Ничего сверх
# 3 *-> Нормирование и сглаживание поверхности, например по Критерию Делоне # 
Не находила алгоритм для в пространстве, хотя пишут и очевидно он есть. Вот целая книга по нему [Книга](./SkvortsovAV-2002-01.Book(Trn).pdf). Тем не менее есть библиотека scipy.spatial и там все реализовано, вот, что получается 

<img src="Снимок экрана 2024-09-17 060914.png" alt="image" width="400"/>
 + [Статья](Fast_Delaunay_triangulation_in_three_dim.pdf)

Соответствующие макросы на FreeCAD - normalize, Delaunay в папке macros

# Что сейчас?  #
Я рассматриваю вариант разбития поверхности на более примитивные, с одним локальным минимумом.
Воспользуемся поиском в глубину.
- Находим самую или одну из самых низко расположенных вершин и начинаем отсчет от нее. С помощью DFS(или BFS), идем по ребрам по мере увеличения координат точек по  OZ. Далее уже продолжая работать с небольшими поверхностями по контуру выделяя соответствующие у нужных изоповерхностей. В теории, на краях поверхности наименьшее расстояние по изоповерхности, а от локального минимум наибольшее, и надо будет выбирать нужную из соответствующего диапазона
Соответствующие макросы на FreeCAD - DFS, но пока рабоатет как-то криво.