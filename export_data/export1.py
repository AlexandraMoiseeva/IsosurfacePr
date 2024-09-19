from QFormAPI import *
script_dir = os.path.dirname(os.path.abspath(__file__))

try:
    qform = QForm()
    qform.qform_dir_set(r'\\Dmitry\TestRelease\QForm\11.0.224.10\x64')

    if qform.is_started_by_qform():
        qform.qform_attach()
    else:
        arg2 = SessionId()
        arg2.session_id = 0
        qform.qform_attach_to(arg2)
        
    ret3:Record = qform.record_get_last()
    
    arg3 = ExportFieldIsosurface()
    arg3.object_type = ObjectType.Workpiece
    arg3.object_id = 1
    arg3.source_operation = 2
    arg3.field = "T"
    for i in range(10):
        arg3.field_value = i/10
        arg3.file = script_dir + r'/data/' + 'isosurface_folder/'+ 'isosurface' + str(i/10) + '.stl'
        isolines = qform.invoke(0, 'export_field_isosurface', arg3, None)
    
    arg4 = MeshExport()
    arg4.object_type = ObjectType.Workpiece
    arg4.object_id = 1
    arg4.format = MeshFormat.STL
    arg4.file = script_dir + r'/data/' + 'mesh'
    qform.invoke(0, 'export_mesh', arg4, None)
        

except Exception as ex:
	print(str(ex))


