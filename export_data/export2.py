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
        
        arg3 = MeshExport()
        arg3.object_type = ObjectType.Workpiece
        arg3.object_id = 1
        arg3.format = MeshFormat.CSV3D
        arg3.file = script_dir + r'/data/' + 'mesh'
        qform.invoke(0, 'export_mesh', arg3, None)
        

except Exception as ex:
	print(str(ex))


