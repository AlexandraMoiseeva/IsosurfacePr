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
        
    arg3 = FileName()
    arg3.file = script_dir + r'/data/mesh.stl'
    ret3:ObjectList = qform.geometry_load(arg3)
    
    for i in range(10)[1:]:
        arg4 = FileName()
        arg4.file = script_dir + r'/data/isosurface_folder/' + 'isosurface' + str(i/10) + '.stl'
        ret4:ObjectList = qform.geometry_load(arg4)
        

except Exception as ex:
	print(str(ex))


