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
    arg3.file = r'C:\\Users\Александра\Documents\GitHub\IsosurfacePr\export_data\data\isosurface.stl'
    ret3:ObjectList = qform.geometry_load(arg3)

        

except Exception as ex:
	print(str(ex))


