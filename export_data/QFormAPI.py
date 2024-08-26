import struct
import io
import os
import subprocess
from typing import List
import urllib.parse
import sys

class BScmType:
	def __init__(self, code, sz, stru):
		self._code = code
		self._size = sz
		self._stru = stru
	def code(self, val): return self._code
	def size(self, val): return self._size
	def write(self, W, val):
		W.write(self._stru.pack(val))
	def read(self, R, cod):
		bs = R.read(self._size)
		(v,) = self._stru.unpack(bs)
		return v

class BScmTypeList:
	def __init__(self, code, sz, stru):
		self._code = code
		self._size = sz
		self._stru = stru
	def code(self, val): return self._code
	def size(self, val): return self._size*len(val) + BScmFmt.SizeInt
	def write(self, W, val):
		BScmFmt.write_int(W, len(val))
		for v in val:
			W.write(self._stru.pack(v))
	def read(self, R, cod):
		n = BScmFmt.read_int(R)
		l = []
		bs = R.read(n*self._size)
		ofs = 0
		while n > 0:
			n -= 1
			(v,) = self._stru.unpack_from(bs, ofs)
			l.append(v)
			ofs += self._size
		return l

class BScmTypeBool:
	def code(self, val): return BScmFmt.BTrue if val else BScmFmt.BFalse
	def size(self, val):	return 0
	def write(self, W, val): pass
	def read(self, R, cod): return True if cod == BScmFmt.BTrue else False

class BScmTypeNone:
	def code(self, val): return BScmFmt.BNil
	def size(self, val):	return 0
	def write(self, W, val): pass
	def read(self, R, cod): return None

class BScmTypeStr:
	def code(self, val): return BScmFmt.BStr
	def size(self, val):
		if val is None: return BScmFmt.SizeInt
		return len(val.encode('utf-8')) + BScmFmt.SizeInt
	def write(self, W, val):
		if val is None:
			BScmFmt.write_int(W, 0)
			return
		bts = val.encode('utf-8')
		BScmFmt.write_int(W, len(bts))
		W.write(bts)
	def read(self, R, cod):
		n = BScmFmt.read_int(R)
		(csz, enc) = (2, 'utf-16') if cod == BScmFmt.BWcs else (1, 'utf-8')
		bs = R.read(n*csz)
		return bs.decode(enc)

class BScmFmt:
	BNil = 1
	BTrue = 2
	BFalse = 3
	BStr = 10
	BWcs = 11
	BBool = 20
	BInt = 21
	BDouble = 22
	BArrayBool = 40
	BArrayInt = 41
	BArrayDouble = 42
	BShift = 128

	StruByte = struct.Struct('B')
	StruInt = struct.Struct('i')
	StruDouble = struct.Struct('d')

	SizeByte = 1
	SizeInt = 4
	SizeDouble = 8

	TypeNone = BScmTypeNone()
	TypeInt = BScmType(BInt, SizeInt, StruInt)
	TypeDouble = BScmType(BDouble, SizeDouble, StruDouble)
	TypeBool = BScmTypeBool()
	TypeStr = BScmTypeStr()
	TypeBoolList = BScmTypeList(BArrayBool, SizeByte, StruByte)
	TypeIntList = BScmTypeList(BArrayInt, SizeInt, StruInt)
	TypeDoubleList = BScmTypeList(BArrayDouble, SizeDouble, StruDouble)

	def value_type(cod):
		if cod == BScmFmt.BNil: return BScmFmt.TypeNone
		if cod == BScmFmt.BTrue: return BScmFmt.TypeBool
		if cod == BScmFmt.BFalse: return BScmFmt.TypeBool
		if cod == BScmFmt.BInt: return BScmFmt.TypeInt
		if cod == BScmFmt.BDouble: return BScmFmt.TypeDouble
		if cod == BScmFmt.BStr: return BScmFmt.TypeStr
		if cod == BScmFmt.BWcs: return BScmFmt.TypeStr
		if cod == BScmFmt.BArrayBool: return BScmFmt.TypeBoolList
		if cod == BScmFmt.BArrayInt: return BScmFmt.TypeIntList
		if cod == BScmFmt.BArrayDouble: return BScmFmt.TypeDoubleList

	def safe_len(a):
		if a is not None: return len(a)
		return 0

	def write_int(W, val):
		W.write(BScmFmt.StruInt.pack(val))

	def write_code(W, val):
		W.write(BScmFmt.StruByte.pack(val))

	def read_code(R):
		bs = R.read(BScmFmt.SizeByte)
		(v,) = BScmFmt.StruByte.unpack(bs)
		return v

	def read_int(R):
		bs = R.read(BScmFmt.SizeInt)
		(v,) = BScmFmt.StruInt.unpack(bs)
		return v

class BScm:
	def __init__(self, v = None, vt = BScmFmt.TypeNone):
		self.value = v
		self.vtype = vt
		self.childs = None

	def value_set_int(self, s):
		self.value = s
		self.vtype = BScmFmt.TypeInt

	def value_set_str(self, s):
		self.value = s
		self.vtype = BScmFmt.TypeStr

	def child_get(self, key):
		if self.childs is None: return None
		for c in self.childs:
			if c.value == key:
				return c
		return None

	def prop_get(self, key, defval = None):
		c = self.child_get(key)
		if c is None: return defval
		if c.childs is None: return defval
		return c.childs[0].value

	def enum_get(self, key, Klass, defval = None):
		sv = self.prop_get(key)
		if sv is None: return defval
		ev = getattr(Klass, sv)
		if ev is None: return defval
		return ev

	def enum_array_get(self, key, Klass):
		c = self.child_get(key)
		if c is None: return []
		if c.childs is None: return []
		ss = []
		for ev in c.childs:
			ss.append(getattr(Klass, ev.value))
		return ss

	def strings_get(self, key):
		c = self.child_get(key)
		if c is None: return []
		if c.childs is None: return []
		ss = []
		for s in c.childs:
			ss.append(s.value)
		return ss

	def objects_get(self, key, Klass):
		c = self.child_get(key)
		if c is None: return []
		if c.childs is None: return []
		os = []
		for s in c.childs:
			k = Klass()
			k.load(s)
			os.append(k)
		return os

	def object_get(self, key, Klass):
		c = self.child_get(key)
		if c is None: return Klass()
		k = Klass()
		k.load(c)
		return k

	def prop_add(self, key, v, vt):
		n = self.add(key, BScmFmt.TypeStr)
		n.add(v, vt)

	def enum_add(self, key, v):
		self.prop_add(key, v.value, BScmFmt.TypeStr)

	def enum_array_add(self, key, ss):
		n = self.add(key, BScmFmt.TypeStr)
		n.childs = []
		for s in ss:
			n.childs.append(BScm(s.value, BScmFmt.TypeStr))

	def add(self, v, vt):
		node = BScm(v, vt)
		if self.childs is None:
			self.childs = []
		self.childs.append(node)
		return node

	def strings_add(self, key, ss):
		n = self.add(key, BScmFmt.TypeStr)
		n.childs = []
		for s in ss:
			n.childs.append(BScm(s, BScmFmt.TypeStr))

	def objects_add(self, key, cs):
		n = self.add(key, BScmFmt.TypeStr)
		n.childs = []
		for c in cs:
			cn = BScm();
			c.store(cn)
			n.childs.append(cn)
            
	def object_add(self, key, obj):
		n = self.add(key, BScmFmt.TypeStr)
		obj.store(n)

	def byte_size(self):
		sz = 1 # code
		sz += self.vtype.size(self.value)
		if BScmFmt.safe_len(self.childs) > 0:
			sz += BScmFmt.SizeInt
			for c in self.childs:
				sz += c.byte_size()
		return sz

	def write(self, W):
		cod = self.vtype.code(self.value)
		n = BScmFmt.safe_len(self.childs)
		if n > 0:
			cod += BScmFmt.BShift
			BScmFmt.write_code(W, cod)
			self.vtype.write(W, self.value)
			BScmFmt.write_int(W, n)
			for chld in self.childs:
				chld.write(W)
		else:
			BScmFmt.write_code(W, cod)
			self.vtype.write(W, self.value)

	def read_value(self, R, cod):
		self.vtype = BScmFmt.value_type(cod)
		self.value = self.vtype.read(R, cod)

	def read(self, R):
		cod = BScmFmt.read_code(R)
		if cod < BScmFmt.BShift:
			self.read_value(R, cod)
		else:
			self.read_value(R, cod - BScmFmt.BShift)
			n = BScmFmt.read_int(R)
			self.childs = []
			while n > 0:
				c = BScm()
				self.childs.append(c)
				c.read(R)
				n -= 1

class FieldSource:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class FieldSource:
	MainSimulation = FieldSource('MainSimulation')
	ToolSimulation = FieldSource('ToolSimulation')
	ToolCoupledSimulation = FieldSource('ToolCoupledSimulation')
	Subroutine = FieldSource('Subroutine')
	PhaseTransformations = FieldSource('PhaseTransformations')
	Sprayer = FieldSource('Sprayer')
	Hardness = FieldSource('Hardness')
	Concentration = FieldSource('Concentration')
	Inductor = FieldSource('Inductor')

class ObjectType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ObjectType:
	Nothing = ObjectType('Nothing')
	Workpiece = ObjectType('Workpiece')
	Tool = ObjectType('Tool')
	AssembledTool = ObjectType('AssembledTool')
	SymmetryPlane = ObjectType('SymmetryPlane')
	BoundaryCondition = ObjectType('BoundaryCondition')
	Domain = ObjectType('Domain')
	StopCondition = ObjectType('StopCondition')
	Subroutine = ObjectType('Subroutine')
	Operation = ObjectType('Operation')
	Process = ObjectType('Process')
	ImportedObject = ObjectType('ImportedObject')
	MainRoll = ObjectType('MainRoll')
	RollMandrell = ObjectType('RollMandrell')
	AxialRoll = ObjectType('AxialRoll')
	GuideRoll = ObjectType('GuideRoll')
	RollPlate = ObjectType('RollPlate')
	RollEdger = ObjectType('RollEdger')
	PressureRoll = ObjectType('PressureRoll')
	ClippingSurface = ObjectType('ClippingSurface')
	TurningSurface = ObjectType('TurningSurface')
	TracingPoint = ObjectType('TracingPoint')
	TracingLines = ObjectType('TracingLines')
	TracingBox = ObjectType('TracingBox')
	TracingSurfLines = ObjectType('TracingSurfLines')
	TracingContours = ObjectType('TracingContours')
	ExtrWorkpiece = ObjectType('ExtrWorkpiece')
	ExtrToolMesh = ObjectType('ExtrToolMesh')
	ExtrTool = ObjectType('ExtrTool')
	ExtrDieholder = ObjectType('ExtrDieholder')
	ExtrMandrell = ObjectType('ExtrMandrell')
	ExtrDieplace = ObjectType('ExtrDieplace')
	ExtrBacker = ObjectType('ExtrBacker')
	ExtrBolster = ObjectType('ExtrBolster')
	ExtrSpreader = ObjectType('ExtrSpreader')
	ExtrSeparator = ObjectType('ExtrSeparator')
	ElectroPusher = ObjectType('ElectroPusher')
	ElectroAnvil = ObjectType('ElectroAnvil')
	ElectroClamp = ObjectType('ElectroClamp')
	CrossRollingTool = ObjectType('CrossRollingTool')
	CrossRollingMandrelTool = ObjectType('CrossRollingMandrelTool')
	CrossRollingLinearTool = ObjectType('CrossRollingLinearTool')
	CrossRollingFreeRotatedTool = ObjectType('CrossRollingFreeRotatedTool')
	Blow = ObjectType('Blow')
	Billet = ObjectType('Billet')
	Pass = ObjectType('Pass')

class StatusCode:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class StatusCode:
	Ok = StatusCode('Ok')
	Canceled = StatusCode('Canceled')
	Failed = StatusCode('Failed')
	Working = StatusCode('Working')
	Idle = StatusCode('Idle')

class CalculationUnit:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class CalculationUnit:
	Nothing = CalculationUnit('Nothing')
	Chain = CalculationUnit('Chain')
	Operation = CalculationUnit('Operation')
	Blow = CalculationUnit('Blow')
	Billet = CalculationUnit('Billet')
	Pass = CalculationUnit('Pass')
	Records = CalculationUnit('Records')
	ProcessTime = CalculationUnit('ProcessTime')
	CalculationTime = CalculationUnit('CalculationTime')

class CalculationMode:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class CalculationMode:
	Chain = CalculationMode('Chain')
	Operation = CalculationMode('Operation')
	Blow = CalculationMode('Blow')
	Billet = CalculationMode('Billet')
	Pass = CalculationMode('Pass')

class SimulationStage:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class SimulationStage:
	Idle = SimulationStage('Idle')
	Working = SimulationStage('Working')
	Preliminary = SimulationStage('Preliminary')
	Cooling = SimulationStage('Cooling')
	Thermal = SimulationStage('Thermal')
	Forwarding = SimulationStage('Forwarding')
	Remeshing = SimulationStage('Remeshing')
	Plastic = SimulationStage('Plastic')
	Cutting = SimulationStage('Cutting')
	Meshing = SimulationStage('Meshing')
	Tool = SimulationStage('Tool')
	Diffusion = SimulationStage('Diffusion')
	GravitationalPos = SimulationStage('GravitationalPos')

class AsyncEventType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class AsyncEventType:
	Idle = AsyncEventType('Idle')
	Done = AsyncEventType('Done')
	NextCalculationUnit = AsyncEventType('NextCalculationUnit')
	NextRecord = AsyncEventType('NextRecord')
	NextStage = AsyncEventType('NextStage')
	Iteration = AsyncEventType('Iteration')
	Timeout = AsyncEventType('Timeout')
	Error = AsyncEventType('Error')
	Message = AsyncEventType('Message')
	Canceled = AsyncEventType('Canceled')

class MessageType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class MessageType:
	info = MessageType('info')
	error = MessageType('error')
	warning = MessageType('warning')

class FieldGroup:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class FieldGroup:
	Workpiece = FieldGroup('Workpiece')
	Tool = FieldGroup('Tool')
	TrackingPoints = FieldGroup('TrackingPoints')

class BilletParam:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class BilletParam:
	BilletTemperature = BilletParam('BilletTemperature')
	TemperatureTaper = BilletParam('TemperatureTaper')
	VelocityValue = BilletParam('VelocityValue')
	ProfileVelocity = BilletParam('ProfileVelocity')
	BilletLength = BilletParam('BilletLength')
	BilletToBilletPause = BilletParam('BilletToBilletPause')
	MaxStroke = BilletParam('MaxStroke')
	ButtEndLength = BilletParam('ButtEndLength')

class BlowParam:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class BlowParam:
	StopConditionValue = BlowParam('StopConditionValue')
	EnergyShare = BlowParam('EnergyShare')
	CoolingInAir = BlowParam('CoolingInAir')
	CoolingInTool = BlowParam('CoolingInTool')
	Feed = BlowParam('Feed')
	VerticalMovement = BlowParam('VerticalMovement')
	Rotation = BlowParam('Rotation')
	RollingToolMotion = BlowParam('RollingToolMotion')
	UpperToolRotationSpeed = BlowParam('UpperToolRotationSpeed')
	LowerToolRotationSpeed = BlowParam('LowerToolRotationSpeed')
	SideToolRotationSpeed = BlowParam('SideToolRotationSpeed')
	UpperToolCrosswiseMovement = BlowParam('UpperToolCrosswiseMovement')

class BCond:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class BCond:
	Nil = BCond('Nil')
	Load = BCond('Load')
	Velocity = BCond('Velocity')
	Temperature = BCond('Temperature')
	HeatFlow = BCond('HeatFlow')
	HeatFlow_s = BCond('HeatFlow_s')
	HeatFlow_v = BCond('HeatFlow_v')
	Fixing = BCond('Fixing')
	Normal = BCond('Normal')
	Bearing = BCond('Bearing')
	Ring = BCond('Ring')
	Fit = BCond('Fit')
	Friction = BCond('Friction')
	Env = BCond('Env')
	Fastener = BCond('Fastener')
	Pressure = BCond('Pressure')
	Manipulator = BCond('Manipulator')
	Rotation = BCond('Rotation')
	Pusher = BCond('Pusher')
	Sprayer = BCond('Sprayer')
	SprayerRectArray = BCond('SprayerRectArray')
	SprayerPolarArray = BCond('SprayerPolarArray')
	SprayerDB = BCond('SprayerDB')
	SprayerRectArrayDB = BCond('SprayerRectArrayDB')
	SprayerPolarArrayDB = BCond('SprayerPolarArrayDB')
	BodyContour = BCond('BodyContour')
	ConstantTemperatureContact = BCond('ConstantTemperatureContact')
	Inductor = BCond('Inductor')
	SurfTempPointCloud = BCond('SurfTempPointCloud')
	Dilation = BCond('Dilation')

class DbStandart:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class DbStandart:
	Default = DbStandart('Default')
	File = DbStandart('File')
	DIN = DbStandart('DIN')
	AISI = DbStandart('AISI')
	GOST = DbStandart('GOST')
	BS = DbStandart('BS')
	JIS = DbStandart('JIS')
	GB = DbStandart('GB')
	ISO = DbStandart('ISO')

class DriveType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class DriveType:
	Unspecified = DriveType('Unspecified')
	Fixed = DriveType('Fixed')
	Free = DriveType('Free')
	MechanicalPresse = DriveType('MechanicalPresse')
	Hydraulic = DriveType('Hydraulic')
	LoadHolder = DriveType('LoadHolder')
	Universal = DriveType('Universal')
	Tabular = DriveType('Tabular')
	ScrewPress = DriveType('ScrewPress')
	Hammer = DriveType('Hammer')
	UniversalV2 = DriveType('UniversalV2')
	Arbitrary = DriveType('Arbitrary')

class PropertyType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class PropertyType:
	Selector = PropertyType('Selector')
	Value = PropertyType('Value')
	DbObject = PropertyType('DbObject')
	Automatic = PropertyType('Automatic')
	Special = PropertyType('Special')
	Inherited = PropertyType('Inherited')
	ConstSpeed = PropertyType('ConstSpeed')
	FreeRotation = PropertyType('FreeRotation')

class DbTableArg:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class DbTableArg:
	Nothing = DbTableArg('Nothing')
	Strain = DbTableArg('Strain')
	StrainRate = DbTableArg('StrainRate')
	Temperature = DbTableArg('Temperature')
	Time = DbTableArg('Time')
	Pressure = DbTableArg('Pressure')
	CharacteristicTime = DbTableArg('CharacteristicTime')
	Angle = DbTableArg('Angle')
	Stroke = DbTableArg('Stroke')
	Distance = DbTableArg('Distance')
	Displacement = DbTableArg('Displacement')
	Thickness = DbTableArg('Thickness')
	Diameter = DbTableArg('Diameter')
	CoolingRate = DbTableArg('CoolingRate')
	ColumnNumber = DbTableArg('ColumnNumber')
	StressTriaxiality = DbTableArg('StressTriaxiality')
	LodeStressParameter = DbTableArg('LodeStressParameter')
	NormalizedMaxPrincipalStress = DbTableArg('NormalizedMaxPrincipalStress')
	MinorTrueStrain = DbTableArg('MinorTrueStrain')
	DwellTemperature = DbTableArg('DwellTemperature')
	InitialGrainSize = DbTableArg('InitialGrainSize')
	DeformationTemperature = DbTableArg('DeformationTemperature')

class LogFormat:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class LogFormat:
	FromFileExtension = LogFormat('FromFileExtension')
	XML = LogFormat('XML')
	SExpr = LogFormat('SExpr')

class Domain:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class Domain:
	Nil = Domain('Nil')
	Friction = Domain('Friction')
	MeshAdaptation = Domain('MeshAdaptation')
	MeshAdaptationTool = Domain('MeshAdaptationTool')
	MeshAdaptationWorkpiece = Domain('MeshAdaptationWorkpiece')
	GeometricalMeshAdaptation = Domain('GeometricalMeshAdaptation')

class BearingContoursFormat:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class BearingContoursFormat:
	FromFileExtension = BearingContoursFormat('FromFileExtension')
	DXF = BearingContoursFormat('DXF')

class IsolinesFormat:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class IsolinesFormat:
	FromFileExtension = IsolinesFormat('FromFileExtension')
	DXF = IsolinesFormat('DXF')

class SectionFormat:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class SectionFormat:
	FromFileExtension = SectionFormat('FromFileExtension')
	STL = SectionFormat('STL')

class LengthUnit:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class LengthUnit:
	Auto = LengthUnit('Auto')
	m = LengthUnit('m')
	mm = LengthUnit('mm')
	cm = LengthUnit('cm')
	inch = LengthUnit('inch')

class TrackingFieldsFormat:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class TrackingFieldsFormat:
	FromFileExtension = TrackingFieldsFormat('FromFileExtension')
	XLS = TrackingFieldsFormat('XLS')

class ValuesOnSheet:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ValuesOnSheet:
	at_current_time = ValuesOnSheet('at_current_time')
	at_time_blow = ValuesOnSheet('at_time_blow')
	at_time_operation = ValuesOnSheet('at_time_operation')
	at_time_chain = ValuesOnSheet('at_time_chain')
	at_point_blow = ValuesOnSheet('at_point_blow')
	at_point_operation = ValuesOnSheet('at_point_operation')
	at_point_chain = ValuesOnSheet('at_point_chain')

class MeshFormat:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class MeshFormat:
	FromFileExtension = MeshFormat('FromFileExtension')
	CSV3D = MeshFormat('CSV3D')
	CSV2D = MeshFormat('CSV2D')
	DXF = MeshFormat('DXF')
	STL = MeshFormat('STL')
	_3MF = MeshFormat('_3MF')
	XLS = MeshFormat('XLS')
	XLSX = MeshFormat('XLSX')

class ProfileSectionFormat:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ProfileSectionFormat:
	FromFileExtension = ProfileSectionFormat('FromFileExtension')
	CSV2D = ProfileSectionFormat('CSV2D')
	DXF = ProfileSectionFormat('DXF')

class RecordsExportMode:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class RecordsExportMode:
	ExportCurrent = RecordsExportMode('ExportCurrent')
	ExportFromCurrent = RecordsExportMode('ExportFromCurrent')
	ExportToCurrent = RecordsExportMode('ExportToCurrent')

class PickTraceBy:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class PickTraceBy:
	TraceNumber = PickTraceBy('TraceNumber')
	NodeNumber = PickTraceBy('NodeNumber')
	SurfaceElementNumber = PickTraceBy('SurfaceElementNumber')

class FillMode:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class FillMode:
	Gradient = FillMode('Gradient')
	Discrete = FillMode('Discrete')
	Isolines = FillMode('Isolines')
	IsolinesMarks = FillMode('IsolinesMarks')

class Colormap:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class Colormap:
	Auto = Colormap('Auto')
	Jet = Colormap('Jet')
	Temp = Colormap('Temp')
	Grey = Colormap('Grey')
	Legacy = Colormap('Legacy')
	Rainbow = Colormap('Rainbow')
	Turbo = Colormap('Turbo')
	Viridis = Colormap('Viridis')
	Plasma = Colormap('Plasma')
	Copper = Colormap('Copper')
	RdYlBu = Colormap('RdYlBu')
	Spectral = Colormap('Spectral')
	Coolwarm = Colormap('Coolwarm')
	Seismic = Colormap('Seismic')

class HistogramBy:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class HistogramBy:
	ByNodes = HistogramBy('ByNodes')
	ByElements = HistogramBy('ByElements')

class Language:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class Language:
	russian = Language('russian')
	german = Language('german')
	spanish = Language('spanish')
	italian = Language('italian')
	japanese = Language('japanese')
	thai = Language('thai')
	chinese = Language('chinese')
	chinese_t = Language('chinese_t')
	polish = Language('polish')
	portuguese = Language('portuguese')
	turkish = Language('turkish')
	english = Language('english')

class FaceType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class FaceType:
	Unspecified = FaceType('Unspecified')
	SymPlane = FaceType('SymPlane')
	ExtrContainer = FaceType('ExtrContainer')
	ExtrDie = FaceType('ExtrDie')
	ExtrProfile = FaceType('ExtrProfile')
	ExtrBearing = FaceType('ExtrBearing')
	ExtrPrechamber = FaceType('ExtrPrechamber')
	ExtrProfileFree = FaceType('ExtrProfileFree')
	ExtrRam = FaceType('ExtrRam')
	ExtrToolToContainer = FaceType('ExtrToolToContainer')
	ExtrCase = FaceType('ExtrCase')
	ExtrPressureRing = FaceType('ExtrPressureRing')
	ExtrBolsterInner = FaceType('ExtrBolsterInner')
	ExtrDieHolder = FaceType('ExtrDieHolder')

class MeshNodeOwnerType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class MeshNodeOwnerType:
	Face = MeshNodeOwnerType('Face')
	Edge = MeshNodeOwnerType('Edge')
	Apex = MeshNodeOwnerType('Apex')
	Volume = MeshNodeOwnerType('Volume')

class MouseButton:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class MouseButton:
	Right = MouseButton('Right')
	Middle = MouseButton('Middle')
	Left = MouseButton('Left')

class DialogButton:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class DialogButton:
	Ok = DialogButton('Ok')
	Cancel = DialogButton('Cancel')
	Yes = DialogButton('Yes')
	No = DialogButton('No')
	Apply = DialogButton('Apply')
	Done = DialogButton('Done')
	Close = DialogButton('Close')
	Retry = DialogButton('Retry')
	Skip = DialogButton('Skip')
	Ignore = DialogButton('Ignore')
	Continue = DialogButton('Continue')
	SaveAs = DialogButton('SaveAs')

class Direction:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class Direction:
	X = Direction('X')
	Y = Direction('Y')
	Z = Direction('Z')

class DisplayModes:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class DisplayModes:
	visible = DisplayModes('visible')
	show_mesh = DisplayModes('show_mesh')
	transparent = DisplayModes('transparent')
	show_laps = DisplayModes('show_laps')
	show_geometric_mesh = DisplayModes('show_geometric_mesh')

class OperationCreationMode:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class OperationCreationMode:
	CreateAsNewProcess = OperationCreationMode('CreateAsNewProcess')
	AddToCurrentChain = OperationCreationMode('AddToCurrentChain')
	InsertAfterParent = OperationCreationMode('InsertAfterParent')

class OperationType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class OperationType:
	none = OperationType('none')
	electric = OperationType('electric')
	deformation = OperationType('deformation')
	thermo = OperationType('thermo')
	extrusion = OperationType('extrusion')
	ring_rolling = OperationType('ring_rolling')
	wheel_rolling = OperationType('wheel_rolling')
	rolling = OperationType('rolling')
	cross_rolling = OperationType('cross_rolling')
	sheet_bulk_forming = OperationType('sheet_bulk_forming')
	reverse_rolling = OperationType('reverse_rolling')
	cyclic_heating = OperationType('cyclic_heating')
	extr_profile_cooling = OperationType('extr_profile_cooling')

class StopCondType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class StopCondType:
	Distance = StopCondType('Distance')
	Time = StopCondType('Time')
	ToolStroke = StopCondType('ToolStroke')
	ToolRotationAxis1 = StopCondType('ToolRotationAxis1')
	ToolRotationAxis2 = StopCondType('ToolRotationAxis2')
	MaxLoad = StopCondType('MaxLoad')
	FinalPosition = StopCondType('FinalPosition')
	FieldValue = StopCondType('FieldValue')
	SolverTime = StopCondType('SolverTime')

class SolidBodyNodeValueCondition_ValueType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class SolidBodyNodeValueCondition_ValueType:
	MIN_VALUE = SolidBodyNodeValueCondition_ValueType('MIN_VALUE')
	MAX_VALUE = SolidBodyNodeValueCondition_ValueType('MAX_VALUE')

class SolidBodyNodeValueCondition_ValueRegionInBody:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class SolidBodyNodeValueCondition_ValueRegionInBody:
	ANY_NODE = SolidBodyNodeValueCondition_ValueRegionInBody('ANY_NODE')
	DEFINED_BODY_VOLUME_PERCENT = SolidBodyNodeValueCondition_ValueRegionInBody('DEFINED_BODY_VOLUME_PERCENT')
	DEFINED_VOLUME_OF_DEFORMATION_ZONE_PERCENT = SolidBodyNodeValueCondition_ValueRegionInBody('DEFINED_VOLUME_OF_DEFORMATION_ZONE_PERCENT')

class SubRoutineType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class SubRoutineType:
	lua = SubRoutineType('lua')
	wear = SubRoutineType('wear')
	strain_tensor = SubRoutineType('strain_tensor')
	stress_tensor = SubRoutineType('stress_tensor')
	stress_tensor_tool = SubRoutineType('stress_tensor_tool')
	strain_tensor_tool = SubRoutineType('strain_tensor_tool')
	strain_rate_tensor = SubRoutineType('strain_rate_tensor')
	press_wp = SubRoutineType('press_wp')
	press_tool = SubRoutineType('press_tool')
	distance_to_surface = SubRoutineType('distance_to_surface')
	global_displacement = SubRoutineType('global_displacement')
	strain_tensor_lagrangian = SubRoutineType('strain_tensor_lagrangian')
	fatigue = SubRoutineType('fatigue')
	fatigueDB = SubRoutineType('fatigueDB')
	debug_q = SubRoutineType('debug_q')
	debug_tl = SubRoutineType('debug_tl')
	PressCenter = SubRoutineType('PressCenter')
	debug_qwp = SubRoutineType('debug_qwp')
	debug_wp = SubRoutineType('debug_wp')
	debug_1 = SubRoutineType('debug_1')
	debug_2 = SubRoutineType('debug_2')
	FLDdiagram = SubRoutineType('FLDdiagram')
	yield_tensor = SubRoutineType('yield_tensor')
	surface_normal = SubRoutineType('surface_normal')
	surface_normal_tool = SubRoutineType('surface_normal_tool')
	gartfield = SubRoutineType('gartfield')
	slippage = SubRoutineType('slippage')
	normal_velocity = SubRoutineType('normal_velocity')
	track = SubRoutineType('track')
	forgingratio = SubRoutineType('forgingratio')
	track3 = SubRoutineType('track3')
	outcrop = SubRoutineType('outcrop')
	adaptation_tool = SubRoutineType('adaptation_tool')
	adaptation_wp = SubRoutineType('adaptation_wp')
	temperature = SubRoutineType('temperature')
	ort = SubRoutineType('ort')
	thickness = SubRoutineType('thickness')
	cylindric = SubRoutineType('cylindric')
	cyl_velocity = SubRoutineType('cyl_velocity')
	gtn_component = SubRoutineType('gtn_component')
	cylindric_tool = SubRoutineType('cylindric_tool')
	wp_damage = SubRoutineType('wp_damage')
	jmak = SubRoutineType('jmak')
	extr_jmak = SubRoutineType('extr_jmak')
	extr_streaking_lines = SubRoutineType('extr_streaking_lines')
	extr_tool_analysis = SubRoutineType('extr_tool_analysis')
	extr_fill_analysis = SubRoutineType('extr_fill_analysis')
	extr_overheating = SubRoutineType('extr_overheating')
	extr_flow_time = SubRoutineType('extr_flow_time')
	extr_seam_quality = SubRoutineType('extr_seam_quality')
	test_cubic = SubRoutineType('test_cubic')
	velocity_gradient = SubRoutineType('velocity_gradient')
	extr_skin = SubRoutineType('extr_skin')
	extr_funnel = SubRoutineType('extr_funnel')
	extr_seam_quality_ext = SubRoutineType('extr_seam_quality_ext')

class SystemOfUnits:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class SystemOfUnits:
	metric = SystemOfUnits('metric')
	english = SystemOfUnits('english')
	SI = SystemOfUnits('SI')

class ViewOptions:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ViewOptions:
	ShowMinMax = ViewOptions('ShowMinMax')
	ShowCADColors = ViewOptions('ShowCADColors')

class DFProperty:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class DFProperty:
	constant = DFProperty('constant')
	table = DFProperty('table')
	lua = DFProperty('lua')

class FitType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class FitType:
	clearance = FitType('clearance')
	interference = FitType('interference')

class MeshAdaptType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class MeshAdaptType:
	none = MeshAdaptType('none')
	adapt = MeshAdaptType('adapt')
	meshsize = MeshAdaptType('meshsize')

class PointCloudCoordinateUnits:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class PointCloudCoordinateUnits:
	m = PointCloudCoordinateUnits('m')
	mm = PointCloudCoordinateUnits('mm')
	inch = PointCloudCoordinateUnits('inch')

class PointCloudTemperatureUnits:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class PointCloudTemperatureUnits:
	K = PointCloudTemperatureUnits('K')
	C = PointCloudTemperatureUnits('C')
	F = PointCloudTemperatureUnits('F')

class POSITIONING_TYPE:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class POSITIONING_TYPE:
	bring_with_retract = POSITIONING_TYPE('bring_with_retract')
	bring_without_retract = POSITIONING_TYPE('bring_without_retract')
	none = POSITIONING_TYPE('none')
	retract_to_contact = POSITIONING_TYPE('retract_to_contact')
	bring_with_retract_sync = POSITIONING_TYPE('bring_with_retract_sync')

class AxisType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class AxisType:
	point = AxisType('point')
	direction = AxisType('direction')

class HEAT_EXCHANGE:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class HEAT_EXCHANGE:
	winkler = HEAT_EXCHANGE('winkler')
	joint = HEAT_EXCHANGE('joint')
	const_temp = HEAT_EXCHANGE('const_temp')
	none = HEAT_EXCHANGE('none')

class AxisBlockType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class AxisBlockType:
	p = AxisBlockType('p')
	v = AxisBlockType('v')
	x = AxisBlockType('x')
	y = AxisBlockType('y')
	z = AxisBlockType('z')

class AxisInputMode:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class AxisInputMode:
	manual = AxisInputMode('manual')
	automatic = AxisInputMode('automatic')

class ClipType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ClipType:
	inner = ClipType('inner')
	outer = ClipType('outer')

class FieldType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class FieldType:
	elem = FieldType('elem')
	scalar = FieldType('scalar')

class ProtectionMode:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ProtectionMode:
	no = ProtectionMode('no')
	close_src = ProtectionMode('close_src')
	encrypted = ProtectionMode('encrypted')

class Scope:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class Scope:
	public_scope = Scope('public_scope')
	private_scope = Scope('private_scope')

class ContourHierarchy:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ContourHierarchy:
	outer = ContourHierarchy('outer')
	inner = ContourHierarchy('inner')

class ContourClipping:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ContourClipping:
	trimming = ContourClipping('trimming')
	clipping = ContourClipping('clipping')

class BoxAxis:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class BoxAxis:
	axis_z = BoxAxis('axis_z')
	axis_y = BoxAxis('axis_y')
	axis_x = BoxAxis('axis_x')

class StopConditionSolverTimeType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class StopConditionSolverTimeType:
	PROCESS = StopConditionSolverTimeType('PROCESS')
	OPERATION = StopConditionSolverTimeType('OPERATION')
	BLOW = StopConditionSolverTimeType('BLOW')

class SSTMode:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class SSTMode:
	nil = SSTMode('nil')
	source = SSTMode('source')
	result = SSTMode('result')
	preparation = SSTMode('preparation')
	mesh_preparation = SSTMode('mesh_preparation')

class TracingBoxType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class TracingBoxType:
	line = TracingBoxType('line')
	point = TracingBoxType('point')

class Current:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class Current:
	direct = Current('direct')
	alternating = Current('alternating')

class ElectricPowerType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ElectricPowerType:
	current = ElectricPowerType('current')
	voltage = ElectricPowerType('voltage')
	power = ElectricPowerType('power')

class SolveMethod:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class SolveMethod:
	implicit_method = SolveMethod('implicit_method')
	explicit_method = SolveMethod('explicit_method')

class ThermoMethod:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ThermoMethod:
	fe = ThermoMethod('fe')
	voronoi = ThermoMethod('voronoi')

class DEF_TYPE:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class DEF_TYPE:
	none = DEF_TYPE('none')
	flat = DEF_TYPE('flat')
	axis = DEF_TYPE('axis')
	_3d = DEF_TYPE('_3d')

class MicrostructureModuleTypes:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class MicrostructureModuleTypes:
	qform = MicrostructureModuleTypes('qform')
	gmt = MicrostructureModuleTypes('gmt')

class ThermalProcessingType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ThermalProcessingType:
	heating = ThermalProcessingType('heating')
	tempering = ThermalProcessingType('tempering')
	quenching = ThermalProcessingType('quenching')

class RRVStabilization:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class RRVStabilization:
	yes = RRVStabilization('yes')
	no = RRVStabilization('no')
	automatic = RRVStabilization('automatic')

class RRMRollVelocityType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class RRMRollVelocityType:
	angular = RRMRollVelocityType('angular')
	linear = RRMRollVelocityType('linear')

class RRGuideRollForceType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class RRGuideRollForceType:
	percent = RRGuideRollForceType('percent')
	formula = RRGuideRollForceType('formula')

class RRFinDiam:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class RRFinDiam:
	max = RRFinDiam('max')
	level = RRFinDiam('level')

class RRStrategy:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class RRStrategy:
	diam_height = RRStrategy('diam_height')
	feed = RRStrategy('feed')
	mandrell_roll = RRStrategy('mandrell_roll')
	mandrell_ring = RRStrategy('mandrell_ring')

class RRAxialHDispType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class RRAxialHDispType:
	hauto = RRAxialHDispType('hauto')
	diam = RRAxialHDispType('diam')
	htime = RRAxialHDispType('htime')
	from_diam = RRAxialHDispType('from_diam')

class RRAxialVDispType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class RRAxialVDispType:
	top = RRAxialVDispType('top')
	sync = RRAxialVDispType('sync')
	ratio = RRAxialVDispType('ratio')

class RRWheelStrat:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class RRWheelStrat:
	rolls = RRWheelStrat('rolls')
	shaft = RRWheelStrat('shaft')

class RollingPusherVelocityType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class RollingPusherVelocityType:
	automatic = RollingPusherVelocityType('automatic')
	manual = RollingPusherVelocityType('manual')

class ExtrProcType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ExtrProcType:
	flow = ExtrProcType('flow')
	billet = ExtrProcType('billet')
	steady = ExtrProcType('steady')

class ExtrVelocityType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ExtrVelocityType:
	ram = ExtrVelocityType('ram')
	profile = ExtrVelocityType('profile')
	ran_var = ExtrVelocityType('ran_var')

class ExtrPrestageType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ExtrPrestageType:
	equal = ExtrPrestageType('equal')
	velocity = ExtrPrestageType('velocity')
	time = ExtrPrestageType('time')

class HardnessEstimationMethod:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class HardnessEstimationMethod:
	volume_fractions = HardnessEstimationMethod('volume_fractions')
	characteristic_time = HardnessEstimationMethod('characteristic_time')

class UltimateStrengthEstimationMethod:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class UltimateStrengthEstimationMethod:
	volume_fractions = UltimateStrengthEstimationMethod('volume_fractions')
	characteristic_time = UltimateStrengthEstimationMethod('characteristic_time')

class ToolJoinDeform:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ToolJoinDeform:
	simple = ToolJoinDeform('simple')
	coupled = ToolJoinDeform('coupled')
	stressed_state = ToolJoinDeform('stressed_state')

class HeatTreatmentModuleTypes:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class HeatTreatmentModuleTypes:
	qform = HeatTreatmentModuleTypes('qform')
	gmt = HeatTreatmentModuleTypes('gmt')

class HardnessScales:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class HardnessScales:
	HV = HardnessScales('HV')
	HB = HardnessScales('HB')
	HBW = HardnessScales('HBW')
	HRA = HardnessScales('HRA')
	HRB = HardnessScales('HRB')
	HRC = HardnessScales('HRC')
	HRD = HardnessScales('HRD')
	HRE = HardnessScales('HRE')
	HRF = HardnessScales('HRF')
	HRG = HardnessScales('HRG')
	HRH = HardnessScales('HRH')
	HRK = HardnessScales('HRK')

class ProfileDistortionTimeStepType:
	def __init__(self, v):
		self.value = v
	def __str__(self) -> str:
		return self.value
class ProfileDistortionTimeStepType:
	constant_time_step = ProfileDistortionTimeStepType('constant_time_step')
	var_time_step_const_layer_thickness = ProfileDistortionTimeStepType('var_time_step_const_layer_thickness')

class FieldId:
	def __init__(self, name:str = '', field:str = '', field_source:FieldSource = FieldSource.MainSimulation, field_type:int = 0, field_target:int = 0, source_object:int = -1, source_operation:int = -1, field_min:float = 0.0, field_max:float = 0.0, units:int = 0):
		self.name = name
		self.field = field
		self.field_source = field_source
		self.field_type = field_type
		self.field_target = field_target
		self.source_object = source_object
		self.source_operation = source_operation
		self.field_min = field_min
		self.field_max = field_max
		self.units = units
	def store(self, B):
		B.prop_add('name', self.name, BScmFmt.TypeStr)
		B.prop_add('field', self.field, BScmFmt.TypeStr)
		B.enum_add('field_source', self.field_source)
		B.prop_add('field_type', self.field_type, BScmFmt.TypeInt)
		B.prop_add('field_target', self.field_target, BScmFmt.TypeInt)
		B.prop_add('source_object', self.source_object, BScmFmt.TypeInt)
		B.prop_add('source_operation', self.source_operation, BScmFmt.TypeInt)
		B.prop_add('field_min', self.field_min, BScmFmt.TypeDouble)
		B.prop_add('field_max', self.field_max, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeInt)
	def load(self, B):
		self.name = B.prop_get('name')
		self.field = B.prop_get('field')
		self.field_source = B.enum_get('field_source', FieldSource)
		self.field_type = B.prop_get('field_type')
		self.field_target = B.prop_get('field_target')
		self.source_object = B.prop_get('source_object')
		self.source_operation = B.prop_get('source_operation')
		self.field_min = B.prop_get('field_min')
		self.field_max = B.prop_get('field_max')
		self.units = B.prop_get('units')

class AssembledTool:
	def __init__(self, id:int = -1, parts:List[int] = []):
		self.id = id
		self.parts = parts.copy()
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('parts', self.parts, BScmFmt.TypeIntList)
	def load(self, B):
		self.id = B.prop_get('id')
		self.parts = B.prop_get('parts')

class ItemId:
	def __init__(self, id:int = 0):
		self.id = id
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')

class ItemList:
	def __init__(self, objects:List[int] = []):
		self.objects = objects.copy()
	def store(self, B):
		B.prop_add('objects', self.objects, BScmFmt.TypeIntList)
	def load(self, B):
		self.objects = B.prop_get('objects')

class ObjectList:
	def __init__(self, objects:List['ObjectId'] = []):
		self.objects = objects
	def store(self, B):
		B.objects_add('objects', self.objects)
	def load(self, B):
		self.objects = B.objects_get('objects', ObjectId)

class ObjectId:
	def __init__(self, type:ObjectType = ObjectType.Nothing, id:int = 0):
		self.type = type
		self.id = id
	def store(self, B):
		B.enum_add('type', self.type)
		B.prop_add('id', self.id, BScmFmt.TypeInt)
	def load(self, B):
		self.type = B.enum_get('type', ObjectType)
		self.id = B.prop_get('id')

class SimulationResult:
	def __init__(self, status:StatusCode = StatusCode.Ok):
		self.status = status
	def store(self, B):
		B.enum_add('status', self.status)
	def load(self, B):
		self.status = B.enum_get('status', StatusCode)

class SubroutineCalculationMode:
	def __init__(self, calculation_mode:CalculationUnit = CalculationUnit.Nothing):
		self.calculation_mode = calculation_mode
	def store(self, B):
		B.enum_add('calculation_mode', self.calculation_mode)
	def load(self, B):
		self.calculation_mode = B.enum_get('calculation_mode', CalculationUnit)

class TrackingCalculationMode:
	def __init__(self, calculation_mode:CalculationUnit = CalculationUnit.Nothing, forward:bool = False, backward:bool = False):
		self.calculation_mode = calculation_mode
		self.forward = forward
		self.backward = backward
	def store(self, B):
		B.enum_add('calculation_mode', self.calculation_mode)
		B.prop_add('forward', self.forward, BScmFmt.TypeBool)
		B.prop_add('backward', self.backward, BScmFmt.TypeBool)
	def load(self, B):
		self.calculation_mode = B.enum_get('calculation_mode', CalculationUnit)
		self.forward = B.prop_get('forward')
		self.backward = B.prop_get('backward')

class SimulationParams:
	def __init__(self, start_from_record:int = -1, remesh_tools:bool = False, stop_at_record:int = 0, max_records:int = 0, stop_at_process_time:float = 0.0, max_process_time:float = 0.0, max_calculation_time:int = 0, calculation_mode:CalculationMode = CalculationMode.Chain):
		self.start_from_record = start_from_record
		self.remesh_tools = remesh_tools
		self.stop_at_record = stop_at_record
		self.max_records = max_records
		self.stop_at_process_time = stop_at_process_time
		self.max_process_time = max_process_time
		self.max_calculation_time = max_calculation_time
		self.calculation_mode = calculation_mode
	def store(self, B):
		B.prop_add('start_from_record', self.start_from_record, BScmFmt.TypeInt)
		B.prop_add('remesh_tools', self.remesh_tools, BScmFmt.TypeBool)
		B.prop_add('stop_at_record', self.stop_at_record, BScmFmt.TypeInt)
		B.prop_add('max_records', self.max_records, BScmFmt.TypeInt)
		B.prop_add('stop_at_process_time', self.stop_at_process_time, BScmFmt.TypeDouble)
		B.prop_add('max_process_time', self.max_process_time, BScmFmt.TypeDouble)
		B.prop_add('max_calculation_time', self.max_calculation_time, BScmFmt.TypeInt)
		B.enum_add('calculation_mode', self.calculation_mode)
	def load(self, B):
		self.start_from_record = B.prop_get('start_from_record')
		self.remesh_tools = B.prop_get('remesh_tools')
		self.stop_at_record = B.prop_get('stop_at_record')
		self.max_records = B.prop_get('max_records')
		self.stop_at_process_time = B.prop_get('stop_at_process_time')
		self.max_process_time = B.prop_get('max_process_time')
		self.max_calculation_time = B.prop_get('max_calculation_time')
		self.calculation_mode = B.enum_get('calculation_mode', CalculationMode)

class AsyncState:
	def __init__(self, working:bool = False, simulation_stage:SimulationStage = SimulationStage.Idle, stage_time:int = 0, stage_counter:int = 0, record:float = 0.0, operation:int = 0, blow:int = 0):
		self.working = working
		self.simulation_stage = simulation_stage
		self.stage_time = stage_time
		self.stage_counter = stage_counter
		self.record = record
		self.operation = operation
		self.blow = blow
	def store(self, B):
		B.prop_add('working', self.working, BScmFmt.TypeBool)
		B.enum_add('simulation_stage', self.simulation_stage)
		B.prop_add('stage_time', self.stage_time, BScmFmt.TypeInt)
		B.prop_add('stage_counter', self.stage_counter, BScmFmt.TypeInt)
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
		B.prop_add('operation', self.operation, BScmFmt.TypeInt)
		B.prop_add('blow', self.blow, BScmFmt.TypeInt)
	def load(self, B):
		self.working = B.prop_get('working')
		self.simulation_stage = B.enum_get('simulation_stage', SimulationStage)
		self.stage_time = B.prop_get('stage_time')
		self.stage_counter = B.prop_get('stage_counter')
		self.record = B.prop_get('record')
		self.operation = B.prop_get('operation')
		self.blow = B.prop_get('blow')

class AsyncWaitingParams:
	def __init__(self, timeout:int = 60, with_simulation_stage_events:bool = False, with_diagnostic_events:bool = False, with_iteration_events:bool = False):
		self.timeout = timeout
		self.with_simulation_stage_events = with_simulation_stage_events
		self.with_diagnostic_events = with_diagnostic_events
		self.with_iteration_events = with_iteration_events
	def store(self, B):
		B.prop_add('timeout', self.timeout, BScmFmt.TypeInt)
		B.prop_add('with_simulation_stage_events', self.with_simulation_stage_events, BScmFmt.TypeBool)
		B.prop_add('with_diagnostic_events', self.with_diagnostic_events, BScmFmt.TypeBool)
		B.prop_add('with_iteration_events', self.with_iteration_events, BScmFmt.TypeBool)
	def load(self, B):
		self.timeout = B.prop_get('timeout')
		self.with_simulation_stage_events = B.prop_get('with_simulation_stage_events')
		self.with_diagnostic_events = B.prop_get('with_diagnostic_events')
		self.with_iteration_events = B.prop_get('with_iteration_events')

class AsyncEvent:
	def __init__(self, working:bool = False, type:AsyncEventType = AsyncEventType.Idle, finished_calculation_unit:CalculationUnit = CalculationUnit.Nothing, simulation_stage:SimulationStage = SimulationStage.Idle, stage_time:int = 0, stage_counter:int = -1, record:float = -1, progress:float = -1, process_time:float = -1, diagnostic_msg:str = '', diagnostic_msg_type:MessageType = MessageType.info, diagnostic_msg_code:int = 0, iteration:int = 0, iteration_velocity_norm:float = 0.0, iteration_mean_stress_norm:float = 0.0, iteration_separated_nodes:int = 0, iteration_sticked_nodes:int = 0, operation:int = 0, blow:int = 0, backward:bool = False, units:str = ''):
		self.working = working
		self.type = type
		self.finished_calculation_unit = finished_calculation_unit
		self.simulation_stage = simulation_stage
		self.stage_time = stage_time
		self.stage_counter = stage_counter
		self.record = record
		self.progress = progress
		self.process_time = process_time
		self.diagnostic_msg = diagnostic_msg
		self.diagnostic_msg_type = diagnostic_msg_type
		self.diagnostic_msg_code = diagnostic_msg_code
		self.iteration = iteration
		self.iteration_velocity_norm = iteration_velocity_norm
		self.iteration_mean_stress_norm = iteration_mean_stress_norm
		self.iteration_separated_nodes = iteration_separated_nodes
		self.iteration_sticked_nodes = iteration_sticked_nodes
		self.operation = operation
		self.blow = blow
		self.backward = backward
		self.units = units
	def store(self, B):
		B.prop_add('working', self.working, BScmFmt.TypeBool)
		B.enum_add('type', self.type)
		B.enum_add('finished_calculation_unit', self.finished_calculation_unit)
		B.enum_add('simulation_stage', self.simulation_stage)
		B.prop_add('stage_time', self.stage_time, BScmFmt.TypeInt)
		B.prop_add('stage_counter', self.stage_counter, BScmFmt.TypeInt)
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
		B.prop_add('progress', self.progress, BScmFmt.TypeDouble)
		B.prop_add('process_time', self.process_time, BScmFmt.TypeDouble)
		B.prop_add('diagnostic_msg', self.diagnostic_msg, BScmFmt.TypeStr)
		B.enum_add('diagnostic_msg_type', self.diagnostic_msg_type)
		B.prop_add('diagnostic_msg_code', self.diagnostic_msg_code, BScmFmt.TypeInt)
		B.prop_add('iteration', self.iteration, BScmFmt.TypeInt)
		B.prop_add('iteration_velocity_norm', self.iteration_velocity_norm, BScmFmt.TypeDouble)
		B.prop_add('iteration_mean_stress_norm', self.iteration_mean_stress_norm, BScmFmt.TypeDouble)
		B.prop_add('iteration_separated_nodes', self.iteration_separated_nodes, BScmFmt.TypeInt)
		B.prop_add('iteration_sticked_nodes', self.iteration_sticked_nodes, BScmFmt.TypeInt)
		B.prop_add('operation', self.operation, BScmFmt.TypeInt)
		B.prop_add('blow', self.blow, BScmFmt.TypeInt)
		B.prop_add('backward', self.backward, BScmFmt.TypeBool)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.working = B.prop_get('working')
		self.type = B.enum_get('type', AsyncEventType)
		self.finished_calculation_unit = B.enum_get('finished_calculation_unit', CalculationUnit)
		self.simulation_stage = B.enum_get('simulation_stage', SimulationStage)
		self.stage_time = B.prop_get('stage_time')
		self.stage_counter = B.prop_get('stage_counter')
		self.record = B.prop_get('record')
		self.progress = B.prop_get('progress')
		self.process_time = B.prop_get('process_time')
		self.diagnostic_msg = B.prop_get('diagnostic_msg')
		self.diagnostic_msg_type = B.enum_get('diagnostic_msg_type', MessageType)
		self.diagnostic_msg_code = B.prop_get('diagnostic_msg_code')
		self.iteration = B.prop_get('iteration')
		self.iteration_velocity_norm = B.prop_get('iteration_velocity_norm')
		self.iteration_mean_stress_norm = B.prop_get('iteration_mean_stress_norm')
		self.iteration_separated_nodes = B.prop_get('iteration_separated_nodes')
		self.iteration_sticked_nodes = B.prop_get('iteration_sticked_nodes')
		self.operation = B.prop_get('operation')
		self.blow = B.prop_get('blow')
		self.backward = B.prop_get('backward')
		self.units = B.prop_get('units')

class FieldGroupId:
	def __init__(self, group:FieldGroup = FieldGroup.Workpiece):
		self.group = group
	def store(self, B):
		B.enum_add('group', self.group)
	def load(self, B):
		self.group = B.enum_get('group', FieldGroup)

class FieldIdList:
	def __init__(self, fields:List['FieldId'] = []):
		self.fields = fields
	def store(self, B):
		B.objects_add('fields', self.fields)
	def load(self, B):
		self.fields = B.objects_get('fields', FieldId)

class ObjectAxis:
	def __init__(self, defined:bool = False, inherited:bool = False, point1_x:float = 0.0, point1_y:float = 0.0, point1_z:float = 0.0, point2_x:float = 0.0, point2_y:float = 0.0, point2_z:float = 0.0, units:str = ''):
		self.defined = defined
		self.inherited = inherited
		self.point1_x = point1_x
		self.point1_y = point1_y
		self.point1_z = point1_z
		self.point2_x = point2_x
		self.point2_y = point2_y
		self.point2_z = point2_z
		self.units = units
	def store(self, B):
		B.prop_add('defined', self.defined, BScmFmt.TypeBool)
		B.prop_add('inherited', self.inherited, BScmFmt.TypeBool)
		B.prop_add('point1_x', self.point1_x, BScmFmt.TypeDouble)
		B.prop_add('point1_y', self.point1_y, BScmFmt.TypeDouble)
		B.prop_add('point1_z', self.point1_z, BScmFmt.TypeDouble)
		B.prop_add('point2_x', self.point2_x, BScmFmt.TypeDouble)
		B.prop_add('point2_y', self.point2_y, BScmFmt.TypeDouble)
		B.prop_add('point2_z', self.point2_z, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.defined = B.prop_get('defined')
		self.inherited = B.prop_get('inherited')
		self.point1_x = B.prop_get('point1_x')
		self.point1_y = B.prop_get('point1_y')
		self.point1_z = B.prop_get('point1_z')
		self.point2_x = B.prop_get('point2_x')
		self.point2_y = B.prop_get('point2_y')
		self.point2_z = B.prop_get('point2_z')
		self.units = B.prop_get('units')

class ObjectAxisId:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, axis:int = 1):
		self.object_type = object_type
		self.object_id = object_id
		self.axis = axis
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('axis', self.axis, BScmFmt.TypeInt)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.axis = B.prop_get('axis')

class ObjectAxisParams:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, axis:int = 1, point1_x:float = 0.0, point1_y:float = 0.0, point1_z:float = 0.0, point2_x:float = 0.0, point2_y:float = 0.0, point2_z:float = 0.0):
		self.object_type = object_type
		self.object_id = object_id
		self.axis = axis
		self.point1_x = point1_x
		self.point1_y = point1_y
		self.point1_z = point1_z
		self.point2_x = point2_x
		self.point2_y = point2_y
		self.point2_z = point2_z
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('axis', self.axis, BScmFmt.TypeInt)
		B.prop_add('point1_x', self.point1_x, BScmFmt.TypeDouble)
		B.prop_add('point1_y', self.point1_y, BScmFmt.TypeDouble)
		B.prop_add('point1_z', self.point1_z, BScmFmt.TypeDouble)
		B.prop_add('point2_x', self.point2_x, BScmFmt.TypeDouble)
		B.prop_add('point2_y', self.point2_y, BScmFmt.TypeDouble)
		B.prop_add('point2_z', self.point2_z, BScmFmt.TypeDouble)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.axis = B.prop_get('axis')
		self.point1_x = B.prop_get('point1_x')
		self.point1_y = B.prop_get('point1_y')
		self.point1_z = B.prop_get('point1_z')
		self.point2_x = B.prop_get('point2_x')
		self.point2_y = B.prop_get('point2_y')
		self.point2_z = B.prop_get('point2_z')

class Count:
	def __init__(self, count:int = 0):
		self.count = count
	def store(self, B):
		B.prop_add('count', self.count, BScmFmt.TypeInt)
	def load(self, B):
		self.count = B.prop_get('count')

class BilletParameter:
	def __init__(self, billet:int = 0, param:BilletParam = BilletParam.BilletTemperature, value:float = 0.0):
		self.billet = billet
		self.param = param
		self.value = value
	def store(self, B):
		B.prop_add('billet', self.billet, BScmFmt.TypeInt)
		B.enum_add('param', self.param)
		B.prop_add('value', self.value, BScmFmt.TypeDouble)
	def load(self, B):
		self.billet = B.prop_get('billet')
		self.param = B.enum_get('param', BilletParam)
		self.value = B.prop_get('value')

class NullableRealValue:
	def __init__(self, value:float = 0.0, is_null:bool = False, units:str = ''):
		self.value = value
		self.is_null = is_null
		self.units = units
	def store(self, B):
		B.prop_add('value', self.value, BScmFmt.TypeDouble)
		B.prop_add('is_null', self.is_null, BScmFmt.TypeBool)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.value = B.prop_get('value')
		self.is_null = B.prop_get('is_null')
		self.units = B.prop_get('units')

class BlowParameter:
	def __init__(self, blow:int = 0, param:BlowParam = BlowParam.StopConditionValue, stop_condition:int = 0, value:float = 0.0):
		self.blow = blow
		self.param = param
		self.stop_condition = stop_condition
		self.value = value
	def store(self, B):
		B.prop_add('blow', self.blow, BScmFmt.TypeInt)
		B.enum_add('param', self.param)
		B.prop_add('stop_condition', self.stop_condition, BScmFmt.TypeInt)
		B.prop_add('value', self.value, BScmFmt.TypeDouble)
	def load(self, B):
		self.blow = B.prop_get('blow')
		self.param = B.enum_get('param', BlowParam)
		self.stop_condition = B.prop_get('stop_condition')
		self.value = B.prop_get('value')

class BoundCondParams:
	def __init__(self, id:int = -1, type:BCond = BCond.Nil, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0):
		self.id = id
		self.type = type
		self.object_type = object_type
		self.object_id = object_id
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('type', self.type)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.type = B.enum_get('type', BCond)
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')

class ShapeBody:
	def __init__(self, id:int = 0, source_body_type:ObjectType = ObjectType.Nothing, source_body_id:int = 0):
		self.id = id
		self.source_body_type = source_body_type
		self.source_body_id = source_body_id
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('source_body_type', self.source_body_type)
		B.prop_add('source_body_id', self.source_body_id, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.source_body_type = B.enum_get('source_body_type', ObjectType)
		self.source_body_id = B.prop_get('source_body_id')

class ShapeBrick:
	def __init__(self, id:int = 0, size1_x:float = 0.0, size1_y:float = 0.0, size1_z:float = 0.0, size2_x:float = 0.0, size2_y:float = 0.0, size2_z:float = 0.0):
		self.id = id
		self.size1_x = size1_x
		self.size1_y = size1_y
		self.size1_z = size1_z
		self.size2_x = size2_x
		self.size2_y = size2_y
		self.size2_z = size2_z
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('size1_x', self.size1_x, BScmFmt.TypeDouble)
		B.prop_add('size1_y', self.size1_y, BScmFmt.TypeDouble)
		B.prop_add('size1_z', self.size1_z, BScmFmt.TypeDouble)
		B.prop_add('size2_x', self.size2_x, BScmFmt.TypeDouble)
		B.prop_add('size2_y', self.size2_y, BScmFmt.TypeDouble)
		B.prop_add('size2_z', self.size2_z, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.size1_x = B.prop_get('size1_x')
		self.size1_y = B.prop_get('size1_y')
		self.size1_z = B.prop_get('size1_z')
		self.size2_x = B.prop_get('size2_x')
		self.size2_y = B.prop_get('size2_y')
		self.size2_z = B.prop_get('size2_z')

class ShapeCircle:
	def __init__(self, id:int = 0, r:float = 0.0, inner_r:float = 0.0):
		self.id = id
		self.r = r
		self.inner_r = inner_r
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('r', self.r, BScmFmt.TypeDouble)
		B.prop_add('inner_r', self.inner_r, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.r = B.prop_get('r')
		self.inner_r = B.prop_get('inner_r')

class ShapeCone:
	def __init__(self, id:int = 0, angle:float = 0.0, d:float = 0.0, h:float = 0.0):
		self.id = id
		self.angle = angle
		self.d = d
		self.h = h
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('angle', self.angle, BScmFmt.TypeDouble)
		B.prop_add('d', self.d, BScmFmt.TypeDouble)
		B.prop_add('h', self.h, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.angle = B.prop_get('angle')
		self.d = B.prop_get('d')
		self.h = B.prop_get('h')

class ShapeCylinder:
	def __init__(self, id:int = 0, r:float = 0.0, inner_r:float = 0.0, h1:float = 0.0, h2:float = 0.0):
		self.id = id
		self.r = r
		self.inner_r = inner_r
		self.h1 = h1
		self.h2 = h2
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('r', self.r, BScmFmt.TypeDouble)
		B.prop_add('inner_r', self.inner_r, BScmFmt.TypeDouble)
		B.prop_add('h1', self.h1, BScmFmt.TypeDouble)
		B.prop_add('h2', self.h2, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.r = B.prop_get('r')
		self.inner_r = B.prop_get('inner_r')
		self.h1 = B.prop_get('h1')
		self.h2 = B.prop_get('h2')

class ShapeRect:
	def __init__(self, id:int = 0, size1_x:float = 0.0, size1_z:float = 0.0, size2_x:float = 0.0, size2_z:float = 0.0):
		self.id = id
		self.size1_x = size1_x
		self.size1_z = size1_z
		self.size2_x = size2_x
		self.size2_z = size2_z
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('size1_x', self.size1_x, BScmFmt.TypeDouble)
		B.prop_add('size1_z', self.size1_z, BScmFmt.TypeDouble)
		B.prop_add('size2_x', self.size2_x, BScmFmt.TypeDouble)
		B.prop_add('size2_z', self.size2_z, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.size1_x = B.prop_get('size1_x')
		self.size1_z = B.prop_get('size1_z')
		self.size2_x = B.prop_get('size2_x')
		self.size2_z = B.prop_get('size2_z')

class ShapeSphere:
	def __init__(self, id:int = 0, r:float = 0.0, inner_r:float = 0.0):
		self.id = id
		self.r = r
		self.inner_r = inner_r
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('r', self.r, BScmFmt.TypeDouble)
		B.prop_add('inner_r', self.inner_r, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.r = B.prop_get('r')
		self.inner_r = B.prop_get('inner_r')

class ShapeSprayerPolarArray:
	def __init__(self, id:int = 0, angle:float = 0.0, d:float = 0.0, h:float = 0.0, SectorRadius:float = 0.0, SectorAngle:float = 0.0, CountConesInSector:int = 0, CountLayers:int = 0, DistanceBetweenLayers:float = 0.0, AngleBetweenConeAxisAndLayerPlane:float = 0.0, TwistAngle:float = 0.0):
		self.id = id
		self.angle = angle
		self.d = d
		self.h = h
		self.SectorRadius = SectorRadius
		self.SectorAngle = SectorAngle
		self.CountConesInSector = CountConesInSector
		self.CountLayers = CountLayers
		self.DistanceBetweenLayers = DistanceBetweenLayers
		self.AngleBetweenConeAxisAndLayerPlane = AngleBetweenConeAxisAndLayerPlane
		self.TwistAngle = TwistAngle
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('angle', self.angle, BScmFmt.TypeDouble)
		B.prop_add('d', self.d, BScmFmt.TypeDouble)
		B.prop_add('h', self.h, BScmFmt.TypeDouble)
		B.prop_add('SectorRadius', self.SectorRadius, BScmFmt.TypeDouble)
		B.prop_add('SectorAngle', self.SectorAngle, BScmFmt.TypeDouble)
		B.prop_add('CountConesInSector', self.CountConesInSector, BScmFmt.TypeInt)
		B.prop_add('CountLayers', self.CountLayers, BScmFmt.TypeInt)
		B.prop_add('DistanceBetweenLayers', self.DistanceBetweenLayers, BScmFmt.TypeDouble)
		B.prop_add('AngleBetweenConeAxisAndLayerPlane', self.AngleBetweenConeAxisAndLayerPlane, BScmFmt.TypeDouble)
		B.prop_add('TwistAngle', self.TwistAngle, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.angle = B.prop_get('angle')
		self.d = B.prop_get('d')
		self.h = B.prop_get('h')
		self.SectorRadius = B.prop_get('SectorRadius')
		self.SectorAngle = B.prop_get('SectorAngle')
		self.CountConesInSector = B.prop_get('CountConesInSector')
		self.CountLayers = B.prop_get('CountLayers')
		self.DistanceBetweenLayers = B.prop_get('DistanceBetweenLayers')
		self.AngleBetweenConeAxisAndLayerPlane = B.prop_get('AngleBetweenConeAxisAndLayerPlane')
		self.TwistAngle = B.prop_get('TwistAngle')

class ShapeSprayerPolarArrayDB:
	def __init__(self, id:int = 0, SectorRadius:float = 0.0, SectorAngle:float = 0.0, CountConesInSector:int = 0, CountLayers:int = 0, DistanceBetweenLayers:float = 0.0, AngleBetweenConeAxisAndLayerPlane:float = 0.0, TwistAngle:float = 0.0):
		self.id = id
		self.SectorRadius = SectorRadius
		self.SectorAngle = SectorAngle
		self.CountConesInSector = CountConesInSector
		self.CountLayers = CountLayers
		self.DistanceBetweenLayers = DistanceBetweenLayers
		self.AngleBetweenConeAxisAndLayerPlane = AngleBetweenConeAxisAndLayerPlane
		self.TwistAngle = TwistAngle
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('SectorRadius', self.SectorRadius, BScmFmt.TypeDouble)
		B.prop_add('SectorAngle', self.SectorAngle, BScmFmt.TypeDouble)
		B.prop_add('CountConesInSector', self.CountConesInSector, BScmFmt.TypeInt)
		B.prop_add('CountLayers', self.CountLayers, BScmFmt.TypeInt)
		B.prop_add('DistanceBetweenLayers', self.DistanceBetweenLayers, BScmFmt.TypeDouble)
		B.prop_add('AngleBetweenConeAxisAndLayerPlane', self.AngleBetweenConeAxisAndLayerPlane, BScmFmt.TypeDouble)
		B.prop_add('TwistAngle', self.TwistAngle, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.SectorRadius = B.prop_get('SectorRadius')
		self.SectorAngle = B.prop_get('SectorAngle')
		self.CountConesInSector = B.prop_get('CountConesInSector')
		self.CountLayers = B.prop_get('CountLayers')
		self.DistanceBetweenLayers = B.prop_get('DistanceBetweenLayers')
		self.AngleBetweenConeAxisAndLayerPlane = B.prop_get('AngleBetweenConeAxisAndLayerPlane')
		self.TwistAngle = B.prop_get('TwistAngle')

class ShapeSprayerRectArray:
	def __init__(self, id:int = 0, angle:float = 0.0, d:float = 0.0, h:float = 0.0, CountConesDir1:int = 0, CountConesDir2:int = 0, DistanceDir1:float = 0.0, DistanceDir2:float = 0.0, TwistAngle:float = 0.0):
		self.id = id
		self.angle = angle
		self.d = d
		self.h = h
		self.CountConesDir1 = CountConesDir1
		self.CountConesDir2 = CountConesDir2
		self.DistanceDir1 = DistanceDir1
		self.DistanceDir2 = DistanceDir2
		self.TwistAngle = TwistAngle
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('angle', self.angle, BScmFmt.TypeDouble)
		B.prop_add('d', self.d, BScmFmt.TypeDouble)
		B.prop_add('h', self.h, BScmFmt.TypeDouble)
		B.prop_add('CountConesDir1', self.CountConesDir1, BScmFmt.TypeInt)
		B.prop_add('CountConesDir2', self.CountConesDir2, BScmFmt.TypeInt)
		B.prop_add('DistanceDir1', self.DistanceDir1, BScmFmt.TypeDouble)
		B.prop_add('DistanceDir2', self.DistanceDir2, BScmFmt.TypeDouble)
		B.prop_add('TwistAngle', self.TwistAngle, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.angle = B.prop_get('angle')
		self.d = B.prop_get('d')
		self.h = B.prop_get('h')
		self.CountConesDir1 = B.prop_get('CountConesDir1')
		self.CountConesDir2 = B.prop_get('CountConesDir2')
		self.DistanceDir1 = B.prop_get('DistanceDir1')
		self.DistanceDir2 = B.prop_get('DistanceDir2')
		self.TwistAngle = B.prop_get('TwistAngle')

class ShapeSprayerRectArrayDB:
	def __init__(self, id:int = 0, CountConesDir1:int = 0, CountConesDir2:int = 0, DistanceDir1:float = 0.0, DistanceDir2:float = 0.0, TwistAngle:float = 0.0):
		self.id = id
		self.CountConesDir1 = CountConesDir1
		self.CountConesDir2 = CountConesDir2
		self.DistanceDir1 = DistanceDir1
		self.DistanceDir2 = DistanceDir2
		self.TwistAngle = TwistAngle
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('CountConesDir1', self.CountConesDir1, BScmFmt.TypeInt)
		B.prop_add('CountConesDir2', self.CountConesDir2, BScmFmt.TypeInt)
		B.prop_add('DistanceDir1', self.DistanceDir1, BScmFmt.TypeDouble)
		B.prop_add('DistanceDir2', self.DistanceDir2, BScmFmt.TypeDouble)
		B.prop_add('TwistAngle', self.TwistAngle, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.CountConesDir1 = B.prop_get('CountConesDir1')
		self.CountConesDir2 = B.prop_get('CountConesDir2')
		self.DistanceDir1 = B.prop_get('DistanceDir1')
		self.DistanceDir2 = B.prop_get('DistanceDir2')
		self.TwistAngle = B.prop_get('TwistAngle')

class ShapeSurfaceByColor:
	def __init__(self, id:int = 0, color_R:int = 0, color_G:int = 0, color_B:int = 0):
		self.id = id
		self.color_R = color_R
		self.color_G = color_G
		self.color_B = color_B
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('color_R', self.color_R, BScmFmt.TypeInt)
		B.prop_add('color_G', self.color_G, BScmFmt.TypeInt)
		B.prop_add('color_B', self.color_B, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.color_R = B.prop_get('color_R')
		self.color_G = B.prop_get('color_G')
		self.color_B = B.prop_get('color_B')

class BoundCondType:
	def __init__(self, type:BCond = BCond.Nil):
		self.type = type
	def store(self, B):
		B.enum_add('type', self.type)
	def load(self, B):
		self.type = B.enum_get('type', BCond)

class ChartId:
	def __init__(self, arg_object_type:ObjectType = ObjectType.Nothing, arg_object_id:int = 0, arg_subobject:int = 0, arg_id:int = 0, func_object_type:ObjectType = ObjectType.Nothing, func_object_id:int = 0, func_subobject:int = 0, func_id:int = 0):
		self.arg_object_type = arg_object_type
		self.arg_object_id = arg_object_id
		self.arg_subobject = arg_subobject
		self.arg_id = arg_id
		self.func_object_type = func_object_type
		self.func_object_id = func_object_id
		self.func_subobject = func_subobject
		self.func_id = func_id
	def store(self, B):
		B.enum_add('arg_object_type', self.arg_object_type)
		B.prop_add('arg_object_id', self.arg_object_id, BScmFmt.TypeInt)
		B.prop_add('arg_subobject', self.arg_subobject, BScmFmt.TypeInt)
		B.prop_add('arg_id', self.arg_id, BScmFmt.TypeInt)
		B.enum_add('func_object_type', self.func_object_type)
		B.prop_add('func_object_id', self.func_object_id, BScmFmt.TypeInt)
		B.prop_add('func_subobject', self.func_subobject, BScmFmt.TypeInt)
		B.prop_add('func_id', self.func_id, BScmFmt.TypeInt)
	def load(self, B):
		self.arg_object_type = B.enum_get('arg_object_type', ObjectType)
		self.arg_object_id = B.prop_get('arg_object_id')
		self.arg_subobject = B.prop_get('arg_subobject')
		self.arg_id = B.prop_get('arg_id')
		self.func_object_type = B.enum_get('func_object_type', ObjectType)
		self.func_object_id = B.prop_get('func_object_id')
		self.func_subobject = B.prop_get('func_subobject')
		self.func_id = B.prop_get('func_id')

class Chart:
	def __init__(self, arg_value:List[float] = [], arg_has_value:List[bool] = [], func_value:List[float] = [], func_has_value:List[bool] = [], func_units:str = '', arg_units:str = '', arg_name:str = '', func_name:str = ''):
		self.arg_value = arg_value.copy()
		self.arg_has_value = arg_has_value.copy()
		self.func_value = func_value.copy()
		self.func_has_value = func_has_value.copy()
		self.func_units = func_units
		self.arg_units = arg_units
		self.arg_name = arg_name
		self.func_name = func_name
	def store(self, B):
		B.prop_add('arg_value', self.arg_value, BScmFmt.TypeDoubleList)
		B.prop_add('arg_has_value', self.arg_has_value, BScmFmt.TypeBoolList)
		B.prop_add('func_value', self.func_value, BScmFmt.TypeDoubleList)
		B.prop_add('func_has_value', self.func_has_value, BScmFmt.TypeBoolList)
		B.prop_add('func_units', self.func_units, BScmFmt.TypeStr)
		B.prop_add('arg_units', self.arg_units, BScmFmt.TypeStr)
		B.prop_add('arg_name', self.arg_name, BScmFmt.TypeStr)
		B.prop_add('func_name', self.func_name, BScmFmt.TypeStr)
	def load(self, B):
		self.arg_value = B.prop_get('arg_value')
		self.arg_has_value = B.prop_get('arg_has_value')
		self.func_value = B.prop_get('func_value')
		self.func_has_value = B.prop_get('func_has_value')
		self.func_units = B.prop_get('func_units')
		self.arg_units = B.prop_get('arg_units')
		self.arg_name = B.prop_get('arg_name')
		self.func_name = B.prop_get('func_name')

class ChartValueId:
	def __init__(self, arg_object_type:ObjectType = ObjectType.Nothing, arg_object_id:int = 0, arg_subobject:int = 0, arg_id:int = 0, func_object_type:ObjectType = ObjectType.Nothing, func_object_id:int = 0, func_subobject:int = 0, func_id:int = 0, record:float = -1):
		self.arg_object_type = arg_object_type
		self.arg_object_id = arg_object_id
		self.arg_subobject = arg_subobject
		self.arg_id = arg_id
		self.func_object_type = func_object_type
		self.func_object_id = func_object_id
		self.func_subobject = func_subobject
		self.func_id = func_id
		self.record = record
	def store(self, B):
		B.enum_add('arg_object_type', self.arg_object_type)
		B.prop_add('arg_object_id', self.arg_object_id, BScmFmt.TypeInt)
		B.prop_add('arg_subobject', self.arg_subobject, BScmFmt.TypeInt)
		B.prop_add('arg_id', self.arg_id, BScmFmt.TypeInt)
		B.enum_add('func_object_type', self.func_object_type)
		B.prop_add('func_object_id', self.func_object_id, BScmFmt.TypeInt)
		B.prop_add('func_subobject', self.func_subobject, BScmFmt.TypeInt)
		B.prop_add('func_id', self.func_id, BScmFmt.TypeInt)
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
	def load(self, B):
		self.arg_object_type = B.enum_get('arg_object_type', ObjectType)
		self.arg_object_id = B.prop_get('arg_object_id')
		self.arg_subobject = B.prop_get('arg_subobject')
		self.arg_id = B.prop_get('arg_id')
		self.func_object_type = B.enum_get('func_object_type', ObjectType)
		self.func_object_id = B.prop_get('func_object_id')
		self.func_subobject = B.prop_get('func_subobject')
		self.func_id = B.prop_get('func_id')
		self.record = B.prop_get('record')

class ChartValue:
	def __init__(self, arg_value:float = 0.0, func_value:float = 0.0, arg_has_value:bool = False, func_has_value:bool = False, record:float = 0.0, func_units:str = '', arg_units:str = '', arg_name:str = '', func_name:str = ''):
		self.arg_value = arg_value
		self.func_value = func_value
		self.arg_has_value = arg_has_value
		self.func_has_value = func_has_value
		self.record = record
		self.func_units = func_units
		self.arg_units = arg_units
		self.arg_name = arg_name
		self.func_name = func_name
	def store(self, B):
		B.prop_add('arg_value', self.arg_value, BScmFmt.TypeDouble)
		B.prop_add('func_value', self.func_value, BScmFmt.TypeDouble)
		B.prop_add('arg_has_value', self.arg_has_value, BScmFmt.TypeBool)
		B.prop_add('func_has_value', self.func_has_value, BScmFmt.TypeBool)
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
		B.prop_add('func_units', self.func_units, BScmFmt.TypeStr)
		B.prop_add('arg_units', self.arg_units, BScmFmt.TypeStr)
		B.prop_add('arg_name', self.arg_name, BScmFmt.TypeStr)
		B.prop_add('func_name', self.func_name, BScmFmt.TypeStr)
	def load(self, B):
		self.arg_value = B.prop_get('arg_value')
		self.func_value = B.prop_get('func_value')
		self.arg_has_value = B.prop_get('arg_has_value')
		self.func_has_value = B.prop_get('func_has_value')
		self.record = B.prop_get('record')
		self.func_units = B.prop_get('func_units')
		self.arg_units = B.prop_get('arg_units')
		self.arg_name = B.prop_get('arg_name')
		self.func_name = B.prop_get('func_name')

class WebAddress:
	def __init__(self, url:str = ''):
		self.url = url
	def store(self, B):
		B.prop_add('url', self.url, BScmFmt.TypeStr)
	def load(self, B):
		self.url = B.prop_get('url')

class ExecutionStatus:
	def __init__(self, successful:bool = False, msg:str = ''):
		self.successful = successful
		self.msg = msg
	def store(self, B):
		B.prop_add('successful', self.successful, BScmFmt.TypeBool)
		B.prop_add('msg', self.msg, BScmFmt.TypeStr)
	def load(self, B):
		self.successful = B.prop_get('successful')
		self.msg = B.prop_get('msg')

class ContactArea:
	def __init__(self, contact_area:float = 0.0, total_area:float = 0.0, units:str = ''):
		self.contact_area = contact_area
		self.total_area = total_area
		self.units = units
	def store(self, B):
		B.prop_add('contact_area', self.contact_area, BScmFmt.TypeDouble)
		B.prop_add('total_area', self.total_area, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.contact_area = B.prop_get('contact_area')
		self.total_area = B.prop_get('total_area')
		self.units = B.prop_get('units')

class FieldContact:
	def __init__(self, type:ObjectType = ObjectType.Nothing, id:int = 0, in_elements:bool = False):
		self.type = type
		self.id = id
		self.in_elements = in_elements
	def store(self, B):
		B.enum_add('type', self.type)
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('in_elements', self.in_elements, BScmFmt.TypeBool)
	def load(self, B):
		self.type = B.enum_get('type', ObjectType)
		self.id = B.prop_get('id')
		self.in_elements = B.prop_get('in_elements')

class Field:
	def __init__(self, values:List[float] = [], has_data:List[bool] = [], only_on_surface:bool = False, in_elements:bool = False, units:str = ''):
		self.values = values.copy()
		self.has_data = has_data.copy()
		self.only_on_surface = only_on_surface
		self.in_elements = in_elements
		self.units = units
	def store(self, B):
		B.prop_add('values', self.values, BScmFmt.TypeDoubleList)
		B.prop_add('has_data', self.has_data, BScmFmt.TypeBoolList)
		B.prop_add('only_on_surface', self.only_on_surface, BScmFmt.TypeBool)
		B.prop_add('in_elements', self.in_elements, BScmFmt.TypeBool)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.values = B.prop_get('values')
		self.has_data = B.prop_get('has_data')
		self.only_on_surface = B.prop_get('only_on_surface')
		self.in_elements = B.prop_get('in_elements')
		self.units = B.prop_get('units')

class DbObjectPath:
	def __init__(self, db_path:str = ''):
		self.db_path = db_path
	def store(self, B):
		B.prop_add('db_path', self.db_path, BScmFmt.TypeStr)
	def load(self, B):
		self.db_path = B.prop_get('db_path')

class DbArbitraryDriveRecords:
	def __init__(self, db_path:str = '', records:List['DbArbitraryDriveRecord'] = []):
		self.db_path = db_path
		self.records = records
	def store(self, B):
		B.prop_add('db_path', self.db_path, BScmFmt.TypeStr)
		B.objects_add('records', self.records)
	def load(self, B):
		self.db_path = B.prop_get('db_path')
		self.records = B.objects_get('records', DbArbitraryDriveRecord)

class DbArbitraryDriveRecord:
	def __init__(self, t:float = 0.0, x:float = 0.0, y:float = 0.0, z:float = 0.0, rx:float = 0.0, ry:float = 0.0, rz:float = 0.0, units:str = ''):
		self.t = t
		self.x = x
		self.y = y
		self.z = z
		self.rx = rx
		self.ry = ry
		self.rz = rz
		self.units = units
	def store(self, B):
		B.prop_add('t', self.t, BScmFmt.TypeDouble)
		B.prop_add('x', self.x, BScmFmt.TypeDouble)
		B.prop_add('y', self.y, BScmFmt.TypeDouble)
		B.prop_add('z', self.z, BScmFmt.TypeDouble)
		B.prop_add('rx', self.rx, BScmFmt.TypeDouble)
		B.prop_add('ry', self.ry, BScmFmt.TypeDouble)
		B.prop_add('rz', self.rz, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.t = B.prop_get('t')
		self.x = B.prop_get('x')
		self.y = B.prop_get('y')
		self.z = B.prop_get('z')
		self.rx = B.prop_get('rx')
		self.ry = B.prop_get('ry')
		self.rz = B.prop_get('rz')
		self.units = B.prop_get('units')

class DbFetchParams:
	def __init__(self, db_name:str = '', db_standart:DbStandart = DbStandart.Default):
		self.db_name = db_name
		self.db_standart = db_standart
	def store(self, B):
		B.prop_add('db_name', self.db_name, BScmFmt.TypeStr)
		B.enum_add('db_standart', self.db_standart)
	def load(self, B):
		self.db_name = B.prop_get('db_name')
		self.db_standart = B.enum_get('db_standart', DbStandart)

class DbItem:
	def __init__(self, name:str = '', db_path:str = '', childs:List['DbItem'] = []):
		self.name = name
		self.db_path = db_path
		self.childs = childs
	def store(self, B):
		B.prop_add('name', self.name, BScmFmt.TypeStr)
		B.prop_add('db_path', self.db_path, BScmFmt.TypeStr)
		B.objects_add('childs', self.childs)
	def load(self, B):
		self.name = B.prop_get('name')
		self.db_path = B.prop_get('db_path')
		self.childs = B.objects_get('childs', DbItem)

class DbObjectCreationParams:
	def __init__(self, path:str = '', drive_type:DriveType = DriveType.Unspecified):
		self.path = path
		self.drive_type = drive_type
	def store(self, B):
		B.prop_add('path', self.path, BScmFmt.TypeStr)
		B.enum_add('drive_type', self.drive_type)
	def load(self, B):
		self.path = B.prop_get('path')
		self.drive_type = B.enum_get('drive_type', DriveType)

class PathName:
	def __init__(self, path:str = ''):
		self.path = path
	def store(self, B):
		B.prop_add('path', self.path, BScmFmt.TypeStr)
	def load(self, B):
		self.path = B.prop_get('path')

class BoolValue:
	def __init__(self, value:bool = False):
		self.value = value
	def store(self, B):
		B.prop_add('value', self.value, BScmFmt.TypeBool)
	def load(self, B):
		self.value = B.prop_get('value')

class SrcTargetPath:
	def __init__(self, source_path:str = '', target_path:str = ''):
		self.source_path = source_path
		self.target_path = target_path
	def store(self, B):
		B.prop_add('source_path', self.source_path, BScmFmt.TypeStr)
		B.prop_add('target_path', self.target_path, BScmFmt.TypeStr)
	def load(self, B):
		self.source_path = B.prop_get('source_path')
		self.target_path = B.prop_get('target_path')

class DbProperty:
	def __init__(self, db_path:str = '', prop_path:str = '', value:str = ''):
		self.db_path = db_path
		self.prop_path = prop_path
		self.value = value
	def store(self, B):
		B.prop_add('db_path', self.db_path, BScmFmt.TypeStr)
		B.prop_add('prop_path', self.prop_path, BScmFmt.TypeStr)
		B.prop_add('value', self.value, BScmFmt.TypeStr)
	def load(self, B):
		self.db_path = B.prop_get('db_path')
		self.prop_path = B.prop_get('prop_path')
		self.value = B.prop_get('value')

class PropertyValue:
	def __init__(self, property_type:PropertyType = PropertyType.Selector, value:str = '', units:str = ''):
		self.property_type = property_type
		self.value = value
		self.units = units
	def store(self, B):
		B.enum_add('property_type', self.property_type)
		B.prop_add('value', self.value, BScmFmt.TypeStr)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.property_type = B.enum_get('property_type', PropertyType)
		self.value = B.prop_get('value')
		self.units = B.prop_get('units')

class DbPropertyTable:
	def __init__(self, db_path:str = '', prop_path:str = '', row_arg_values:List[float] = [], column_arg_values:List[float] = [], layer_arg_values:List[float] = [], row_arg:DbTableArg = DbTableArg.Nothing, column_arg:DbTableArg = DbTableArg.Nothing, layer_arg:DbTableArg = DbTableArg.Nothing, values:List[float] = [], units:str = ''):
		self.db_path = db_path
		self.prop_path = prop_path
		self.row_arg_values = row_arg_values.copy()
		self.column_arg_values = column_arg_values.copy()
		self.layer_arg_values = layer_arg_values.copy()
		self.row_arg = row_arg
		self.column_arg = column_arg
		self.layer_arg = layer_arg
		self.values = values.copy()
		self.units = units
	def store(self, B):
		B.prop_add('db_path', self.db_path, BScmFmt.TypeStr)
		B.prop_add('prop_path', self.prop_path, BScmFmt.TypeStr)
		B.prop_add('row_arg_values', self.row_arg_values, BScmFmt.TypeDoubleList)
		B.prop_add('column_arg_values', self.column_arg_values, BScmFmt.TypeDoubleList)
		B.prop_add('layer_arg_values', self.layer_arg_values, BScmFmt.TypeDoubleList)
		B.enum_add('row_arg', self.row_arg)
		B.enum_add('column_arg', self.column_arg)
		B.enum_add('layer_arg', self.layer_arg)
		B.prop_add('values', self.values, BScmFmt.TypeDoubleList)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.db_path = B.prop_get('db_path')
		self.prop_path = B.prop_get('prop_path')
		self.row_arg_values = B.prop_get('row_arg_values')
		self.column_arg_values = B.prop_get('column_arg_values')
		self.layer_arg_values = B.prop_get('layer_arg_values')
		self.row_arg = B.enum_get('row_arg', DbTableArg)
		self.column_arg = B.enum_get('column_arg', DbTableArg)
		self.layer_arg = B.enum_get('layer_arg', DbTableArg)
		self.values = B.prop_get('values')
		self.units = B.prop_get('units')

class BatchParams:
	def __init__(self, file:str = '', log_file:str = '', log_format:LogFormat = LogFormat.FromFileExtension, log_input:bool = False, log_output:bool = False):
		self.file = file
		self.log_file = log_file
		self.log_format = log_format
		self.log_input = log_input
		self.log_output = log_output
	def store(self, B):
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.prop_add('log_file', self.log_file, BScmFmt.TypeStr)
		B.enum_add('log_format', self.log_format)
		B.prop_add('log_input', self.log_input, BScmFmt.TypeBool)
		B.prop_add('log_output', self.log_output, BScmFmt.TypeBool)
	def load(self, B):
		self.file = B.prop_get('file')
		self.log_file = B.prop_get('log_file')
		self.log_format = B.enum_get('log_format', LogFormat)
		self.log_input = B.prop_get('log_input')
		self.log_output = B.prop_get('log_output')

class DomainParams:
	def __init__(self, id:int = -1, type:Domain = Domain.Nil, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0):
		self.id = id
		self.type = type
		self.object_type = object_type
		self.object_id = object_id
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('type', self.type)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.type = B.enum_get('type', Domain)
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')

class DomainType:
	def __init__(self, type:Domain = Domain.Nil):
		self.type = type
	def store(self, B):
		B.enum_add('type', self.type)
	def load(self, B):
		self.type = B.enum_get('type', Domain)

class FileName:
	def __init__(self, file:str = ''):
		self.file = file
	def store(self, B):
		B.prop_add('file', self.file, BScmFmt.TypeStr)
	def load(self, B):
		self.file = B.prop_get('file')

class Contours:
	def __init__(self, contours:List['Contour'] = []):
		self.contours = contours
	def store(self, B):
		B.objects_add('contours', self.contours)
	def load(self, B):
		self.contours = B.objects_get('contours', Contour)

class Contour:
	def __init__(self, name:str = '', point_x:List[float] = [], point_y:List[float] = [], point_z:List[float] = [], closed:bool = False, units:str = ''):
		self.name = name
		self.point_x = point_x.copy()
		self.point_y = point_y.copy()
		self.point_z = point_z.copy()
		self.closed = closed
		self.units = units
	def store(self, B):
		B.prop_add('name', self.name, BScmFmt.TypeStr)
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDoubleList)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDoubleList)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDoubleList)
		B.prop_add('closed', self.closed, BScmFmt.TypeBool)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.name = B.prop_get('name')
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.closed = B.prop_get('closed')
		self.units = B.prop_get('units')

class BearingContoursExport:
	def __init__(self, file:str = '', format:BearingContoursFormat = BearingContoursFormat.FromFileExtension):
		self.file = file
		self.format = format
	def store(self, B):
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.enum_add('format', self.format)
	def load(self, B):
		self.file = B.prop_get('file')
		self.format = B.enum_get('format', BearingContoursFormat)

class ExportFieldIsolines:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, field:str = '', field_source:FieldSource = FieldSource.MainSimulation, source_object:int = -1, source_operation:int = -1, field_value:float = 0.0, file:str = '', format:IsolinesFormat = IsolinesFormat.FromFileExtension):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.field = field
		self.field_source = field_source
		self.source_object = source_object
		self.source_operation = source_operation
		self.field_value = field_value
		self.file = file
		self.format = format
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('field', self.field, BScmFmt.TypeStr)
		B.enum_add('field_source', self.field_source)
		B.prop_add('source_object', self.source_object, BScmFmt.TypeInt)
		B.prop_add('source_operation', self.source_operation, BScmFmt.TypeInt)
		B.prop_add('field_value', self.field_value, BScmFmt.TypeDouble)
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.enum_add('format', self.format)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.field = B.prop_get('field')
		self.field_source = B.enum_get('field_source', FieldSource)
		self.source_object = B.prop_get('source_object')
		self.source_operation = B.prop_get('source_operation')
		self.field_value = B.prop_get('field_value')
		self.file = B.prop_get('file')
		self.format = B.enum_get('format', IsolinesFormat)

class ExportFieldIsosurface:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, field:str = '', field_source:FieldSource = FieldSource.MainSimulation, source_object:int = -1, source_operation:int = -1, field_value:float = 0.0, file:str = '', format:SectionFormat = SectionFormat.FromFileExtension, mesh_units:LengthUnit = LengthUnit.Auto):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.field = field
		self.field_source = field_source
		self.source_object = source_object
		self.source_operation = source_operation
		self.field_value = field_value
		self.file = file
		self.format = format
		self.mesh_units = mesh_units
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('field', self.field, BScmFmt.TypeStr)
		B.enum_add('field_source', self.field_source)
		B.prop_add('source_object', self.source_object, BScmFmt.TypeInt)
		B.prop_add('source_operation', self.source_operation, BScmFmt.TypeInt)
		B.prop_add('field_value', self.field_value, BScmFmt.TypeDouble)
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.enum_add('format', self.format)
		B.enum_add('mesh_units', self.mesh_units)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.field = B.prop_get('field')
		self.field_source = B.enum_get('field_source', FieldSource)
		self.source_object = B.prop_get('source_object')
		self.source_operation = B.prop_get('source_operation')
		self.field_value = B.prop_get('field_value')
		self.file = B.prop_get('file')
		self.format = B.enum_get('format', SectionFormat)
		self.mesh_units = B.enum_get('mesh_units', LengthUnit)

class ExportFieldsAtTrackingPoints:
	def __init__(self, file:str = '', file_format:TrackingFieldsFormat = TrackingFieldsFormat.FromFileExtension, values_on_sheet:ValuesOnSheet = ValuesOnSheet.at_point_blow, with_workpiece_points:bool = True, with_tool_points:bool = False):
		self.file = file
		self.file_format = file_format
		self.values_on_sheet = values_on_sheet
		self.with_workpiece_points = with_workpiece_points
		self.with_tool_points = with_tool_points
	def store(self, B):
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.enum_add('file_format', self.file_format)
		B.enum_add('values_on_sheet', self.values_on_sheet)
		B.prop_add('with_workpiece_points', self.with_workpiece_points, BScmFmt.TypeBool)
		B.prop_add('with_tool_points', self.with_tool_points, BScmFmt.TypeBool)
	def load(self, B):
		self.file = B.prop_get('file')
		self.file_format = B.enum_get('file_format', TrackingFieldsFormat)
		self.values_on_sheet = B.enum_get('values_on_sheet', ValuesOnSheet)
		self.with_workpiece_points = B.prop_get('with_workpiece_points')
		self.with_tool_points = B.prop_get('with_tool_points')

class MeshExport:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, file:str = '', format:MeshFormat = MeshFormat.FromFileExtension, mesh_index:int = 0, surface_only:bool = False):
		self.object_type = object_type
		self.object_id = object_id
		self.file = file
		self.format = format
		self.mesh_index = mesh_index
		self.surface_only = surface_only
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.enum_add('format', self.format)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('surface_only', self.surface_only, BScmFmt.TypeBool)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.file = B.prop_get('file')
		self.format = B.enum_get('format', MeshFormat)
		self.mesh_index = B.prop_get('mesh_index')
		self.surface_only = B.prop_get('surface_only')

class ProfileSectionExport:
	def __init__(self, file:str = '', format:ProfileSectionFormat = ProfileSectionFormat.FromFileExtension):
		self.file = file
		self.format = format
	def store(self, B):
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.enum_add('format', self.format)
	def load(self, B):
		self.file = B.prop_get('file')
		self.format = B.enum_get('format', ProfileSectionFormat)

class RecordsExport:
	def __init__(self, mode:RecordsExportMode = RecordsExportMode.ExportCurrent, output_directory:str = ''):
		self.mode = mode
		self.output_directory = output_directory
	def store(self, B):
		B.enum_add('mode', self.mode)
		B.prop_add('output_directory', self.output_directory, BScmFmt.TypeStr)
	def load(self, B):
		self.mode = B.enum_get('mode', RecordsExportMode)
		self.output_directory = B.prop_get('output_directory')

class ExportImage:
	def __init__(self, file:str = '', width:int = 0, height:int = 0, keep_aspect_ratio:bool = False, display_operation_name:bool = False, display_blow_number:bool = False, display_record_number:bool = False, display_time:bool = False, display_time_step:bool = False, zoom_to_fit:bool = False, display_legend:bool = False, legend_width:int = 0):
		self.file = file
		self.width = width
		self.height = height
		self.keep_aspect_ratio = keep_aspect_ratio
		self.display_operation_name = display_operation_name
		self.display_blow_number = display_blow_number
		self.display_record_number = display_record_number
		self.display_time = display_time
		self.display_time_step = display_time_step
		self.zoom_to_fit = zoom_to_fit
		self.display_legend = display_legend
		self.legend_width = legend_width
	def store(self, B):
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.prop_add('width', self.width, BScmFmt.TypeInt)
		B.prop_add('height', self.height, BScmFmt.TypeInt)
		B.prop_add('keep_aspect_ratio', self.keep_aspect_ratio, BScmFmt.TypeBool)
		B.prop_add('display_operation_name', self.display_operation_name, BScmFmt.TypeBool)
		B.prop_add('display_blow_number', self.display_blow_number, BScmFmt.TypeBool)
		B.prop_add('display_record_number', self.display_record_number, BScmFmt.TypeBool)
		B.prop_add('display_time', self.display_time, BScmFmt.TypeBool)
		B.prop_add('display_time_step', self.display_time_step, BScmFmt.TypeBool)
		B.prop_add('zoom_to_fit', self.zoom_to_fit, BScmFmt.TypeBool)
		B.prop_add('display_legend', self.display_legend, BScmFmt.TypeBool)
		B.prop_add('legend_width', self.legend_width, BScmFmt.TypeInt)
	def load(self, B):
		self.file = B.prop_get('file')
		self.width = B.prop_get('width')
		self.height = B.prop_get('height')
		self.keep_aspect_ratio = B.prop_get('keep_aspect_ratio')
		self.display_operation_name = B.prop_get('display_operation_name')
		self.display_blow_number = B.prop_get('display_blow_number')
		self.display_record_number = B.prop_get('display_record_number')
		self.display_time = B.prop_get('display_time')
		self.display_time_step = B.prop_get('display_time_step')
		self.zoom_to_fit = B.prop_get('zoom_to_fit')
		self.display_legend = B.prop_get('display_legend')
		self.legend_width = B.prop_get('legend_width')

class ExportSection:
	def __init__(self, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0, normal_x:float = 0.0, normal_y:float = 0.0, normal_z:float = 0.0, defined:bool = False, units:str = '', object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, u_vector_defined:bool = False, u_vector_x:float = 0.0, u_vector_y:float = 0.0, u_vector_z:float = 0.0, file:str = '', format:SectionFormat = SectionFormat.FromFileExtension, mesh_units:LengthUnit = LengthUnit.Auto):
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
		self.normal_x = normal_x
		self.normal_y = normal_y
		self.normal_z = normal_z
		self.defined = defined
		self.units = units
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.u_vector_defined = u_vector_defined
		self.u_vector_x = u_vector_x
		self.u_vector_y = u_vector_y
		self.u_vector_z = u_vector_z
		self.file = file
		self.format = format
		self.mesh_units = mesh_units
	def store(self, B):
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
		B.prop_add('normal_x', self.normal_x, BScmFmt.TypeDouble)
		B.prop_add('normal_y', self.normal_y, BScmFmt.TypeDouble)
		B.prop_add('normal_z', self.normal_z, BScmFmt.TypeDouble)
		B.prop_add('defined', self.defined, BScmFmt.TypeBool)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('u_vector_defined', self.u_vector_defined, BScmFmt.TypeBool)
		B.prop_add('u_vector_x', self.u_vector_x, BScmFmt.TypeDouble)
		B.prop_add('u_vector_y', self.u_vector_y, BScmFmt.TypeDouble)
		B.prop_add('u_vector_z', self.u_vector_z, BScmFmt.TypeDouble)
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.enum_add('format', self.format)
		B.enum_add('mesh_units', self.mesh_units)
	def load(self, B):
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.normal_x = B.prop_get('normal_x')
		self.normal_y = B.prop_get('normal_y')
		self.normal_z = B.prop_get('normal_z')
		self.defined = B.prop_get('defined')
		self.units = B.prop_get('units')
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.u_vector_defined = B.prop_get('u_vector_defined')
		self.u_vector_x = B.prop_get('u_vector_x')
		self.u_vector_y = B.prop_get('u_vector_y')
		self.u_vector_z = B.prop_get('u_vector_z')
		self.file = B.prop_get('file')
		self.format = B.enum_get('format', SectionFormat)
		self.mesh_units = B.enum_get('mesh_units', LengthUnit)

class TraceId:
	def __init__(self, pick_by:PickTraceBy = PickTraceBy.TraceNumber, number:int = 0):
		self.pick_by = pick_by
		self.number = number
	def store(self, B):
		B.enum_add('pick_by', self.pick_by)
		B.prop_add('number', self.number, BScmFmt.TypeInt)
	def load(self, B):
		self.pick_by = B.enum_get('pick_by', PickTraceBy)
		self.number = B.prop_get('number')

class Trace:
	def __init__(self, number:int = 0, subroutine_fields:List['TraceSubroutineField'] = [], points:List['TracePoint'] = [], units:str = ''):
		self.number = number
		self.subroutine_fields = subroutine_fields
		self.points = points
		self.units = units
	def store(self, B):
		B.prop_add('number', self.number, BScmFmt.TypeInt)
		B.objects_add('subroutine_fields', self.subroutine_fields)
		B.objects_add('points', self.points)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.number = B.prop_get('number')
		self.subroutine_fields = B.objects_get('subroutine_fields', TraceSubroutineField)
		self.points = B.objects_get('points', TracePoint)
		self.units = B.prop_get('units')

class TraceSubroutineField:
	def __init__(self, subroutine:str = '', subroutine_id:int = 0, field:str = '', units:str = '', value_index:int = 0):
		self.subroutine = subroutine
		self.subroutine_id = subroutine_id
		self.field = field
		self.units = units
		self.value_index = value_index
	def store(self, B):
		B.prop_add('subroutine', self.subroutine, BScmFmt.TypeStr)
		B.prop_add('subroutine_id', self.subroutine_id, BScmFmt.TypeInt)
		B.prop_add('field', self.field, BScmFmt.TypeStr)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
		B.prop_add('value_index', self.value_index, BScmFmt.TypeInt)
	def load(self, B):
		self.subroutine = B.prop_get('subroutine')
		self.subroutine_id = B.prop_get('subroutine_id')
		self.field = B.prop_get('field')
		self.units = B.prop_get('units')
		self.value_index = B.prop_get('value_index')

class TracePoint:
	def __init__(self, time:float = 0.0, point_path:float = 0.0, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0, velocity:float = 0.0, velocity_x:float = 0.0, velocity_y:float = 0.0, velocity_z:float = 0.0, temperature:float = 0.0, mean_stress:float = 0.0, effective_stress:float = 0.0, plastic_strain:float = 0.0, strain_rate:float = 0.0, subroutine_fields:List[float] = [], node_1:int = 0, node_2:int = 0, node_3:int = 0, node_4:int = 0, node_1_weigth:float = 0.0, node_2_weigth:float = 0.0, node_3_weigth:float = 0.0, node_4_weigth:float = 0.0):
		self.time = time
		self.point_path = point_path
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
		self.velocity = velocity
		self.velocity_x = velocity_x
		self.velocity_y = velocity_y
		self.velocity_z = velocity_z
		self.temperature = temperature
		self.mean_stress = mean_stress
		self.effective_stress = effective_stress
		self.plastic_strain = plastic_strain
		self.strain_rate = strain_rate
		self.subroutine_fields = subroutine_fields.copy()
		self.node_1 = node_1
		self.node_2 = node_2
		self.node_3 = node_3
		self.node_4 = node_4
		self.node_1_weigth = node_1_weigth
		self.node_2_weigth = node_2_weigth
		self.node_3_weigth = node_3_weigth
		self.node_4_weigth = node_4_weigth
	def store(self, B):
		B.prop_add('time', self.time, BScmFmt.TypeDouble)
		B.prop_add('point_path', self.point_path, BScmFmt.TypeDouble)
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
		B.prop_add('velocity', self.velocity, BScmFmt.TypeDouble)
		B.prop_add('velocity_x', self.velocity_x, BScmFmt.TypeDouble)
		B.prop_add('velocity_y', self.velocity_y, BScmFmt.TypeDouble)
		B.prop_add('velocity_z', self.velocity_z, BScmFmt.TypeDouble)
		B.prop_add('temperature', self.temperature, BScmFmt.TypeDouble)
		B.prop_add('mean_stress', self.mean_stress, BScmFmt.TypeDouble)
		B.prop_add('effective_stress', self.effective_stress, BScmFmt.TypeDouble)
		B.prop_add('plastic_strain', self.plastic_strain, BScmFmt.TypeDouble)
		B.prop_add('strain_rate', self.strain_rate, BScmFmt.TypeDouble)
		B.prop_add('subroutine_fields', self.subroutine_fields, BScmFmt.TypeDoubleList)
		B.prop_add('node_1', self.node_1, BScmFmt.TypeInt)
		B.prop_add('node_2', self.node_2, BScmFmt.TypeInt)
		B.prop_add('node_3', self.node_3, BScmFmt.TypeInt)
		B.prop_add('node_4', self.node_4, BScmFmt.TypeInt)
		B.prop_add('node_1_weigth', self.node_1_weigth, BScmFmt.TypeDouble)
		B.prop_add('node_2_weigth', self.node_2_weigth, BScmFmt.TypeDouble)
		B.prop_add('node_3_weigth', self.node_3_weigth, BScmFmt.TypeDouble)
		B.prop_add('node_4_weigth', self.node_4_weigth, BScmFmt.TypeDouble)
	def load(self, B):
		self.time = B.prop_get('time')
		self.point_path = B.prop_get('point_path')
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.velocity = B.prop_get('velocity')
		self.velocity_x = B.prop_get('velocity_x')
		self.velocity_y = B.prop_get('velocity_y')
		self.velocity_z = B.prop_get('velocity_z')
		self.temperature = B.prop_get('temperature')
		self.mean_stress = B.prop_get('mean_stress')
		self.effective_stress = B.prop_get('effective_stress')
		self.plastic_strain = B.prop_get('plastic_strain')
		self.strain_rate = B.prop_get('strain_rate')
		self.subroutine_fields = B.prop_get('subroutine_fields')
		self.node_1 = B.prop_get('node_1')
		self.node_2 = B.prop_get('node_2')
		self.node_3 = B.prop_get('node_3')
		self.node_4 = B.prop_get('node_4')
		self.node_1_weigth = B.prop_get('node_1_weigth')
		self.node_2_weigth = B.prop_get('node_2_weigth')
		self.node_3_weigth = B.prop_get('node_3_weigth')
		self.node_4_weigth = B.prop_get('node_4_weigth')

class FieldAtMeshPoint:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, field:str = '', field_source:FieldSource = FieldSource.MainSimulation, source_object:int = -1, source_operation:int = -1, node:List[int] = [], node_weight:List[float] = [], on_surface:bool = False):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.field = field
		self.field_source = field_source
		self.source_object = source_object
		self.source_operation = source_operation
		self.node = node.copy()
		self.node_weight = node_weight.copy()
		self.on_surface = on_surface
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('field', self.field, BScmFmt.TypeStr)
		B.enum_add('field_source', self.field_source)
		B.prop_add('source_object', self.source_object, BScmFmt.TypeInt)
		B.prop_add('source_operation', self.source_operation, BScmFmt.TypeInt)
		B.prop_add('node', self.node, BScmFmt.TypeIntList)
		B.prop_add('node_weight', self.node_weight, BScmFmt.TypeDoubleList)
		B.prop_add('on_surface', self.on_surface, BScmFmt.TypeBool)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.field = B.prop_get('field')
		self.field_source = B.enum_get('field_source', FieldSource)
		self.source_object = B.prop_get('source_object')
		self.source_operation = B.prop_get('source_operation')
		self.node = B.prop_get('node')
		self.node_weight = B.prop_get('node_weight')
		self.on_surface = B.prop_get('on_surface')

class RealValue:
	def __init__(self, value:float = 0.0, units:str = ''):
		self.value = value
		self.units = units
	def store(self, B):
		B.prop_add('value', self.value, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.value = B.prop_get('value')
		self.units = B.prop_get('units')

class VectorValue:
	def __init__(self, x:float = 0.0, y:float = 0.0, z:float = 0.0, units:str = ''):
		self.x = x
		self.y = y
		self.z = z
		self.units = units
	def store(self, B):
		B.prop_add('x', self.x, BScmFmt.TypeDouble)
		B.prop_add('y', self.y, BScmFmt.TypeDouble)
		B.prop_add('z', self.z, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.x = B.prop_get('x')
		self.y = B.prop_get('y')
		self.z = B.prop_get('z')
		self.units = B.prop_get('units')

class FieldAtPoint:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, field:str = '', field_source:FieldSource = FieldSource.MainSimulation, source_object:int = -1, source_operation:int = -1, x:float = 0.0, y:float = 0.0, z:float = 0.0, on_surface:bool = False):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.field = field
		self.field_source = field_source
		self.source_object = source_object
		self.source_operation = source_operation
		self.x = x
		self.y = y
		self.z = z
		self.on_surface = on_surface
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('field', self.field, BScmFmt.TypeStr)
		B.enum_add('field_source', self.field_source)
		B.prop_add('source_object', self.source_object, BScmFmt.TypeInt)
		B.prop_add('source_operation', self.source_operation, BScmFmt.TypeInt)
		B.prop_add('x', self.x, BScmFmt.TypeDouble)
		B.prop_add('y', self.y, BScmFmt.TypeDouble)
		B.prop_add('z', self.z, BScmFmt.TypeDouble)
		B.prop_add('on_surface', self.on_surface, BScmFmt.TypeBool)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.field = B.prop_get('field')
		self.field_source = B.enum_get('field_source', FieldSource)
		self.source_object = B.prop_get('source_object')
		self.source_operation = B.prop_get('source_operation')
		self.x = B.prop_get('x')
		self.y = B.prop_get('y')
		self.z = B.prop_get('z')
		self.on_surface = B.prop_get('on_surface')

class FieldAtTrackingObject:
	def __init__(self, object_id:int = 0, object_operation:int = -1, mesh_index:int = 0, field:str = '', field_source:FieldSource = FieldSource.MainSimulation, source_object:int = -1, source_operation:int = -1):
		self.object_id = object_id
		self.object_operation = object_operation
		self.mesh_index = mesh_index
		self.field = field
		self.field_source = field_source
		self.source_object = source_object
		self.source_operation = source_operation
	def store(self, B):
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('object_operation', self.object_operation, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('field', self.field, BScmFmt.TypeStr)
		B.enum_add('field_source', self.field_source)
		B.prop_add('source_object', self.source_object, BScmFmt.TypeInt)
		B.prop_add('source_operation', self.source_operation, BScmFmt.TypeInt)
	def load(self, B):
		self.object_id = B.prop_get('object_id')
		self.object_operation = B.prop_get('object_operation')
		self.mesh_index = B.prop_get('mesh_index')
		self.field = B.prop_get('field')
		self.field_source = B.enum_get('field_source', FieldSource)
		self.source_object = B.prop_get('source_object')
		self.source_operation = B.prop_get('source_operation')

class RealValues:
	def __init__(self, values:List[float] = [], units:str = ''):
		self.values = values.copy()
		self.units = units
	def store(self, B):
		B.prop_add('values', self.values, BScmFmt.TypeDoubleList)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.values = B.prop_get('values')
		self.units = B.prop_get('units')

class VectorValues:
	def __init__(self, x:List[float] = [], y:List[float] = [], z:List[float] = [], units:str = ''):
		self.x = x.copy()
		self.y = y.copy()
		self.z = z.copy()
		self.units = units
	def store(self, B):
		B.prop_add('x', self.x, BScmFmt.TypeDoubleList)
		B.prop_add('y', self.y, BScmFmt.TypeDoubleList)
		B.prop_add('z', self.z, BScmFmt.TypeDoubleList)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.x = B.prop_get('x')
		self.y = B.prop_get('y')
		self.z = B.prop_get('z')
		self.units = B.prop_get('units')

class FieldAtMesh:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, field:str = '', field_source:FieldSource = FieldSource.MainSimulation, source_object:int = -1, source_operation:int = -1):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.field = field
		self.field_source = field_source
		self.source_object = source_object
		self.source_operation = source_operation
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('field', self.field, BScmFmt.TypeStr)
		B.enum_add('field_source', self.field_source)
		B.prop_add('source_object', self.source_object, BScmFmt.TypeInt)
		B.prop_add('source_operation', self.source_operation, BScmFmt.TypeInt)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.field = B.prop_get('field')
		self.field_source = B.enum_get('field_source', FieldSource)
		self.source_object = B.prop_get('source_object')
		self.source_operation = B.prop_get('source_operation')

class VectorField:
	def __init__(self, x:List[float] = [], y:List[float] = [], z:List[float] = [], has_data:List[bool] = [], only_on_surface:bool = False, units:str = ''):
		self.x = x.copy()
		self.y = y.copy()
		self.z = z.copy()
		self.has_data = has_data.copy()
		self.only_on_surface = only_on_surface
		self.units = units
	def store(self, B):
		B.prop_add('x', self.x, BScmFmt.TypeDoubleList)
		B.prop_add('y', self.y, BScmFmt.TypeDoubleList)
		B.prop_add('z', self.z, BScmFmt.TypeDoubleList)
		B.prop_add('has_data', self.has_data, BScmFmt.TypeBoolList)
		B.prop_add('only_on_surface', self.only_on_surface, BScmFmt.TypeBool)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.x = B.prop_get('x')
		self.y = B.prop_get('y')
		self.z = B.prop_get('z')
		self.has_data = B.prop_get('has_data')
		self.only_on_surface = B.prop_get('only_on_surface')
		self.units = B.prop_get('units')

class FieldIsosurface:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, field:str = '', field_source:FieldSource = FieldSource.MainSimulation, source_object:int = -1, source_operation:int = -1, field_value:float = 0.0):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.field = field
		self.field_source = field_source
		self.source_object = source_object
		self.source_operation = source_operation
		self.field_value = field_value
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('field', self.field, BScmFmt.TypeStr)
		B.enum_add('field_source', self.field_source)
		B.prop_add('source_object', self.source_object, BScmFmt.TypeInt)
		B.prop_add('source_operation', self.source_operation, BScmFmt.TypeInt)
		B.prop_add('field_value', self.field_value, BScmFmt.TypeDouble)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.field = B.prop_get('field')
		self.field_source = B.enum_get('field_source', FieldSource)
		self.source_object = B.prop_get('source_object')
		self.source_operation = B.prop_get('source_operation')
		self.field_value = B.prop_get('field_value')

class IsolineList:
	def __init__(self, isolines:List['Isoline'] = []):
		self.isolines = isolines
	def store(self, B):
		B.objects_add('isolines', self.isolines)
	def load(self, B):
		self.isolines = B.objects_get('isolines', Isoline)

class Isoline:
	def __init__(self, coord_x:List[float] = [], coord_z:List[float] = [], source_node_1:List[int] = [], source_node_2:List[int] = [], intersection_t:List[float] = [], closed:bool = False):
		self.coord_x = coord_x.copy()
		self.coord_z = coord_z.copy()
		self.source_node_1 = source_node_1.copy()
		self.source_node_2 = source_node_2.copy()
		self.intersection_t = intersection_t.copy()
		self.closed = closed
	def store(self, B):
		B.prop_add('coord_x', self.coord_x, BScmFmt.TypeDoubleList)
		B.prop_add('coord_z', self.coord_z, BScmFmt.TypeDoubleList)
		B.prop_add('source_node_1', self.source_node_1, BScmFmt.TypeIntList)
		B.prop_add('source_node_2', self.source_node_2, BScmFmt.TypeIntList)
		B.prop_add('intersection_t', self.intersection_t, BScmFmt.TypeDoubleList)
		B.prop_add('closed', self.closed, BScmFmt.TypeBool)
	def load(self, B):
		self.coord_x = B.prop_get('coord_x')
		self.coord_z = B.prop_get('coord_z')
		self.source_node_1 = B.prop_get('source_node_1')
		self.source_node_2 = B.prop_get('source_node_2')
		self.intersection_t = B.prop_get('intersection_t')
		self.closed = B.prop_get('closed')

class IsosurfaceList:
	def __init__(self, isosurfaces:List['Isosurface'] = []):
		self.isosurfaces = isosurfaces
	def store(self, B):
		B.objects_add('isosurfaces', self.isosurfaces)
	def load(self, B):
		self.isosurfaces = B.objects_get('isosurfaces', Isosurface)

class Isosurface:
	def __init__(self, coord_x:List[float] = [], coord_y:List[float] = [], coord_z:List[float] = [], source_node_1:List[int] = [], source_node_2:List[int] = [], intersection_t:List[float] = [], triangle_node_1:List[int] = [], triangle_node_2:List[int] = [], triangle_node_3:List[int] = [], units:str = ''):
		self.coord_x = coord_x.copy()
		self.coord_y = coord_y.copy()
		self.coord_z = coord_z.copy()
		self.source_node_1 = source_node_1.copy()
		self.source_node_2 = source_node_2.copy()
		self.intersection_t = intersection_t.copy()
		self.triangle_node_1 = triangle_node_1.copy()
		self.triangle_node_2 = triangle_node_2.copy()
		self.triangle_node_3 = triangle_node_3.copy()
		self.units = units
	def store(self, B):
		B.prop_add('coord_x', self.coord_x, BScmFmt.TypeDoubleList)
		B.prop_add('coord_y', self.coord_y, BScmFmt.TypeDoubleList)
		B.prop_add('coord_z', self.coord_z, BScmFmt.TypeDoubleList)
		B.prop_add('source_node_1', self.source_node_1, BScmFmt.TypeIntList)
		B.prop_add('source_node_2', self.source_node_2, BScmFmt.TypeIntList)
		B.prop_add('intersection_t', self.intersection_t, BScmFmt.TypeDoubleList)
		B.prop_add('triangle_node_1', self.triangle_node_1, BScmFmt.TypeIntList)
		B.prop_add('triangle_node_2', self.triangle_node_2, BScmFmt.TypeIntList)
		B.prop_add('triangle_node_3', self.triangle_node_3, BScmFmt.TypeIntList)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.coord_x = B.prop_get('coord_x')
		self.coord_y = B.prop_get('coord_y')
		self.coord_z = B.prop_get('coord_z')
		self.source_node_1 = B.prop_get('source_node_1')
		self.source_node_2 = B.prop_get('source_node_2')
		self.intersection_t = B.prop_get('intersection_t')
		self.triangle_node_1 = B.prop_get('triangle_node_1')
		self.triangle_node_2 = B.prop_get('triangle_node_2')
		self.triangle_node_3 = B.prop_get('triangle_node_3')
		self.units = B.prop_get('units')

class FieldAtMinMax:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, field:str = '', field_source:FieldSource = FieldSource.MainSimulation, source_object:int = -1, source_operation:int = -1, on_surface:bool = False):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.field = field
		self.field_source = field_source
		self.source_object = source_object
		self.source_operation = source_operation
		self.on_surface = on_surface
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('field', self.field, BScmFmt.TypeStr)
		B.enum_add('field_source', self.field_source)
		B.prop_add('source_object', self.source_object, BScmFmt.TypeInt)
		B.prop_add('source_operation', self.source_operation, BScmFmt.TypeInt)
		B.prop_add('on_surface', self.on_surface, BScmFmt.TypeBool)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.field = B.prop_get('field')
		self.field_source = B.enum_get('field_source', FieldSource)
		self.source_object = B.prop_get('source_object')
		self.source_operation = B.prop_get('source_operation')
		self.on_surface = B.prop_get('on_surface')

class FieldMinMax:
	def __init__(self, min_value:float = 0.0, min_x:float = 0.0, min_y:float = 0.0, min_z:float = 0.0, min_node:int = -1, max_value:float = 0.0, max_x:float = 0.0, max_y:float = 0.0, max_z:float = 0.0, max_node:int = -1, has_values:bool = False, units:str = ''):
		self.min_value = min_value
		self.min_x = min_x
		self.min_y = min_y
		self.min_z = min_z
		self.min_node = min_node
		self.max_value = max_value
		self.max_x = max_x
		self.max_y = max_y
		self.max_z = max_z
		self.max_node = max_node
		self.has_values = has_values
		self.units = units
	def store(self, B):
		B.prop_add('min_value', self.min_value, BScmFmt.TypeDouble)
		B.prop_add('min_x', self.min_x, BScmFmt.TypeDouble)
		B.prop_add('min_y', self.min_y, BScmFmt.TypeDouble)
		B.prop_add('min_z', self.min_z, BScmFmt.TypeDouble)
		B.prop_add('min_node', self.min_node, BScmFmt.TypeInt)
		B.prop_add('max_value', self.max_value, BScmFmt.TypeDouble)
		B.prop_add('max_x', self.max_x, BScmFmt.TypeDouble)
		B.prop_add('max_y', self.max_y, BScmFmt.TypeDouble)
		B.prop_add('max_z', self.max_z, BScmFmt.TypeDouble)
		B.prop_add('max_node', self.max_node, BScmFmt.TypeInt)
		B.prop_add('has_values', self.has_values, BScmFmt.TypeBool)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.min_value = B.prop_get('min_value')
		self.min_x = B.prop_get('min_x')
		self.min_y = B.prop_get('min_y')
		self.min_z = B.prop_get('min_z')
		self.min_node = B.prop_get('min_node')
		self.max_value = B.prop_get('max_value')
		self.max_x = B.prop_get('max_x')
		self.max_y = B.prop_get('max_y')
		self.max_z = B.prop_get('max_z')
		self.max_node = B.prop_get('max_node')
		self.has_values = B.prop_get('has_values')
		self.units = B.prop_get('units')

class FieldMode:
	def __init__(self, mode:FillMode = FillMode.Gradient):
		self.mode = mode
	def store(self, B):
		B.enum_add('mode', self.mode)
	def load(self, B):
		self.mode = B.enum_get('mode', FillMode)

class FieldPalette:
	def __init__(self, palette:Colormap = Colormap.Auto, inverse:bool = False):
		self.palette = palette
		self.inverse = inverse
	def store(self, B):
		B.enum_add('palette', self.palette)
		B.prop_add('inverse', self.inverse, BScmFmt.TypeBool)
	def load(self, B):
		self.palette = B.enum_get('palette', Colormap)
		self.inverse = B.prop_get('inverse')

class FieldStatAtMesh:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, field:str = '', field_source:FieldSource = FieldSource.MainSimulation, source_object:int = -1, source_operation:int = -1, interval_count:int = 25, histogram_by:HistogramBy = HistogramBy.ByNodes, percentile_1_level:float = 5, percentile_2_level:float = 95):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.field = field
		self.field_source = field_source
		self.source_object = source_object
		self.source_operation = source_operation
		self.interval_count = interval_count
		self.histogram_by = histogram_by
		self.percentile_1_level = percentile_1_level
		self.percentile_2_level = percentile_2_level
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('field', self.field, BScmFmt.TypeStr)
		B.enum_add('field_source', self.field_source)
		B.prop_add('source_object', self.source_object, BScmFmt.TypeInt)
		B.prop_add('source_operation', self.source_operation, BScmFmt.TypeInt)
		B.prop_add('interval_count', self.interval_count, BScmFmt.TypeInt)
		B.enum_add('histogram_by', self.histogram_by)
		B.prop_add('percentile_1_level', self.percentile_1_level, BScmFmt.TypeDouble)
		B.prop_add('percentile_2_level', self.percentile_2_level, BScmFmt.TypeDouble)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.field = B.prop_get('field')
		self.field_source = B.enum_get('field_source', FieldSource)
		self.source_object = B.prop_get('source_object')
		self.source_operation = B.prop_get('source_operation')
		self.interval_count = B.prop_get('interval_count')
		self.histogram_by = B.enum_get('histogram_by', HistogramBy)
		self.percentile_1_level = B.prop_get('percentile_1_level')
		self.percentile_2_level = B.prop_get('percentile_2_level')

class FieldStat:
	def __init__(self, area:float = 0.0, volume:float = 0.0, min_value:float = 0.0, max_value:float = 0.0, mean_value:float = 0.0, standart_deviation:float = 0.0, median:float = 0.0, coefficient_of_skewness:float = 0.0, excess_kurtosis:float = 0.0, percentile_1:float = 0.0, percentile_2:float = 0.0, histogram_field:List[float] = [], histogram_level:List[float] = [], units:str = ''):
		self.area = area
		self.volume = volume
		self.min_value = min_value
		self.max_value = max_value
		self.mean_value = mean_value
		self.standart_deviation = standart_deviation
		self.median = median
		self.coefficient_of_skewness = coefficient_of_skewness
		self.excess_kurtosis = excess_kurtosis
		self.percentile_1 = percentile_1
		self.percentile_2 = percentile_2
		self.histogram_field = histogram_field.copy()
		self.histogram_level = histogram_level.copy()
		self.units = units
	def store(self, B):
		B.prop_add('area', self.area, BScmFmt.TypeDouble)
		B.prop_add('volume', self.volume, BScmFmt.TypeDouble)
		B.prop_add('min_value', self.min_value, BScmFmt.TypeDouble)
		B.prop_add('max_value', self.max_value, BScmFmt.TypeDouble)
		B.prop_add('mean_value', self.mean_value, BScmFmt.TypeDouble)
		B.prop_add('standart_deviation', self.standart_deviation, BScmFmt.TypeDouble)
		B.prop_add('median', self.median, BScmFmt.TypeDouble)
		B.prop_add('coefficient_of_skewness', self.coefficient_of_skewness, BScmFmt.TypeDouble)
		B.prop_add('excess_kurtosis', self.excess_kurtosis, BScmFmt.TypeDouble)
		B.prop_add('percentile_1', self.percentile_1, BScmFmt.TypeDouble)
		B.prop_add('percentile_2', self.percentile_2, BScmFmt.TypeDouble)
		B.prop_add('histogram_field', self.histogram_field, BScmFmt.TypeDoubleList)
		B.prop_add('histogram_level', self.histogram_level, BScmFmt.TypeDoubleList)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.area = B.prop_get('area')
		self.volume = B.prop_get('volume')
		self.min_value = B.prop_get('min_value')
		self.max_value = B.prop_get('max_value')
		self.mean_value = B.prop_get('mean_value')
		self.standart_deviation = B.prop_get('standart_deviation')
		self.median = B.prop_get('median')
		self.coefficient_of_skewness = B.prop_get('coefficient_of_skewness')
		self.excess_kurtosis = B.prop_get('excess_kurtosis')
		self.percentile_1 = B.prop_get('percentile_1')
		self.percentile_2 = B.prop_get('percentile_2')
		self.histogram_field = B.prop_get('histogram_field')
		self.histogram_level = B.prop_get('histogram_level')
		self.units = B.prop_get('units')

class FieldStatAtSection:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, field:str = '', field_source:FieldSource = FieldSource.MainSimulation, source_object:int = -1, source_operation:int = -1, section_point_x:float = 0.0, section_point_y:float = 0.0, section_point_z:float = 0.0, section_normal_x:float = 0.0, section_normal_y:float = 0.0, section_normal_z:float = 0.0, interval_count:int = 25, percentile_1_level:float = 5, percentile_2_level:float = 95):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.field = field
		self.field_source = field_source
		self.source_object = source_object
		self.source_operation = source_operation
		self.section_point_x = section_point_x
		self.section_point_y = section_point_y
		self.section_point_z = section_point_z
		self.section_normal_x = section_normal_x
		self.section_normal_y = section_normal_y
		self.section_normal_z = section_normal_z
		self.interval_count = interval_count
		self.percentile_1_level = percentile_1_level
		self.percentile_2_level = percentile_2_level
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('field', self.field, BScmFmt.TypeStr)
		B.enum_add('field_source', self.field_source)
		B.prop_add('source_object', self.source_object, BScmFmt.TypeInt)
		B.prop_add('source_operation', self.source_operation, BScmFmt.TypeInt)
		B.prop_add('section_point_x', self.section_point_x, BScmFmt.TypeDouble)
		B.prop_add('section_point_y', self.section_point_y, BScmFmt.TypeDouble)
		B.prop_add('section_point_z', self.section_point_z, BScmFmt.TypeDouble)
		B.prop_add('section_normal_x', self.section_normal_x, BScmFmt.TypeDouble)
		B.prop_add('section_normal_y', self.section_normal_y, BScmFmt.TypeDouble)
		B.prop_add('section_normal_z', self.section_normal_z, BScmFmt.TypeDouble)
		B.prop_add('interval_count', self.interval_count, BScmFmt.TypeInt)
		B.prop_add('percentile_1_level', self.percentile_1_level, BScmFmt.TypeDouble)
		B.prop_add('percentile_2_level', self.percentile_2_level, BScmFmt.TypeDouble)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.field = B.prop_get('field')
		self.field_source = B.enum_get('field_source', FieldSource)
		self.source_object = B.prop_get('source_object')
		self.source_operation = B.prop_get('source_operation')
		self.section_point_x = B.prop_get('section_point_x')
		self.section_point_y = B.prop_get('section_point_y')
		self.section_point_z = B.prop_get('section_point_z')
		self.section_normal_x = B.prop_get('section_normal_x')
		self.section_normal_y = B.prop_get('section_normal_y')
		self.section_normal_z = B.prop_get('section_normal_z')
		self.interval_count = B.prop_get('interval_count')
		self.percentile_1_level = B.prop_get('percentile_1_level')
		self.percentile_2_level = B.prop_get('percentile_2_level')

class FileDlg:
	def __init__(self, open_mode:bool = True, file_name:str = '', file_ext:str = '', filters:List[str] = [], current_filter:int = 0, current_folder:str = '', allow_multiple_files:bool = False, settings_key:str = ''):
		self.open_mode = open_mode
		self.file_name = file_name
		self.file_ext = file_ext
		self.filters = filters.copy()
		self.current_filter = current_filter
		self.current_folder = current_folder
		self.allow_multiple_files = allow_multiple_files
		self.settings_key = settings_key
	def store(self, B):
		B.prop_add('open_mode', self.open_mode, BScmFmt.TypeBool)
		B.prop_add('file_name', self.file_name, BScmFmt.TypeStr)
		B.prop_add('file_ext', self.file_ext, BScmFmt.TypeStr)
		B.strings_add('filters', self.filters)
		B.prop_add('current_filter', self.current_filter, BScmFmt.TypeInt)
		B.prop_add('current_folder', self.current_folder, BScmFmt.TypeStr)
		B.prop_add('allow_multiple_files', self.allow_multiple_files, BScmFmt.TypeBool)
		B.prop_add('settings_key', self.settings_key, BScmFmt.TypeStr)
	def load(self, B):
		self.open_mode = B.prop_get('open_mode')
		self.file_name = B.prop_get('file_name')
		self.file_ext = B.prop_get('file_ext')
		self.filters = B.strings_get('filters')
		self.current_filter = B.prop_get('current_filter')
		self.current_folder = B.prop_get('current_folder')
		self.allow_multiple_files = B.prop_get('allow_multiple_files')
		self.settings_key = B.prop_get('settings_key')

class StringList:
	def __init__(self, items:List[str] = []):
		self.items = items.copy()
	def store(self, B):
		B.strings_add('items', self.items)
	def load(self, B):
		self.items = B.strings_get('items')

class ConvertTo3d:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, thickness:float = 0.0, layer_count:int = 0, dir_x:float = 0.0, dir_y:float = 0.0, dir_z:float = 0.0):
		self.object_type = object_type
		self.object_id = object_id
		self.thickness = thickness
		self.layer_count = layer_count
		self.dir_x = dir_x
		self.dir_y = dir_y
		self.dir_z = dir_z
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('thickness', self.thickness, BScmFmt.TypeDouble)
		B.prop_add('layer_count', self.layer_count, BScmFmt.TypeInt)
		B.prop_add('dir_x', self.dir_x, BScmFmt.TypeDouble)
		B.prop_add('dir_y', self.dir_y, BScmFmt.TypeDouble)
		B.prop_add('dir_z', self.dir_z, BScmFmt.TypeDouble)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.thickness = B.prop_get('thickness')
		self.layer_count = B.prop_get('layer_count')
		self.dir_x = B.prop_get('dir_x')
		self.dir_y = B.prop_get('dir_y')
		self.dir_z = B.prop_get('dir_z')

class BrickObjectParams:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, offset_x:float = 0.0, offset_y:float = 0.0, offset_z:float = 0.0, size_x:float = 0.0, size_y:float = 0.0, size_z:float = 0.0, hexahedral_mesh:bool = False, elem_count_x:int = 0, elem_count_y:int = 0, elem_count_z:int = 0):
		self.object_type = object_type
		self.object_id = object_id
		self.offset_x = offset_x
		self.offset_y = offset_y
		self.offset_z = offset_z
		self.size_x = size_x
		self.size_y = size_y
		self.size_z = size_z
		self.hexahedral_mesh = hexahedral_mesh
		self.elem_count_x = elem_count_x
		self.elem_count_y = elem_count_y
		self.elem_count_z = elem_count_z
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('offset_x', self.offset_x, BScmFmt.TypeDouble)
		B.prop_add('offset_y', self.offset_y, BScmFmt.TypeDouble)
		B.prop_add('offset_z', self.offset_z, BScmFmt.TypeDouble)
		B.prop_add('size_x', self.size_x, BScmFmt.TypeDouble)
		B.prop_add('size_y', self.size_y, BScmFmt.TypeDouble)
		B.prop_add('size_z', self.size_z, BScmFmt.TypeDouble)
		B.prop_add('hexahedral_mesh', self.hexahedral_mesh, BScmFmt.TypeBool)
		B.prop_add('elem_count_x', self.elem_count_x, BScmFmt.TypeInt)
		B.prop_add('elem_count_y', self.elem_count_y, BScmFmt.TypeInt)
		B.prop_add('elem_count_z', self.elem_count_z, BScmFmt.TypeInt)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.offset_x = B.prop_get('offset_x')
		self.offset_y = B.prop_get('offset_y')
		self.offset_z = B.prop_get('offset_z')
		self.size_x = B.prop_get('size_x')
		self.size_y = B.prop_get('size_y')
		self.size_z = B.prop_get('size_z')
		self.hexahedral_mesh = B.prop_get('hexahedral_mesh')
		self.elem_count_x = B.prop_get('elem_count_x')
		self.elem_count_y = B.prop_get('elem_count_y')
		self.elem_count_z = B.prop_get('elem_count_z')

class RectObjectParams:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, offset_x:float = 0.0, offset_y:float = 0.0, offset_z:float = 0.0, size_x:float = 0.0, size_z:float = 0.0):
		self.object_type = object_type
		self.object_id = object_id
		self.offset_x = offset_x
		self.offset_y = offset_y
		self.offset_z = offset_z
		self.size_x = size_x
		self.size_z = size_z
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('offset_x', self.offset_x, BScmFmt.TypeDouble)
		B.prop_add('offset_y', self.offset_y, BScmFmt.TypeDouble)
		B.prop_add('offset_z', self.offset_z, BScmFmt.TypeDouble)
		B.prop_add('size_x', self.size_x, BScmFmt.TypeDouble)
		B.prop_add('size_z', self.size_z, BScmFmt.TypeDouble)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.offset_x = B.prop_get('offset_x')
		self.offset_y = B.prop_get('offset_y')
		self.offset_z = B.prop_get('offset_z')
		self.size_x = B.prop_get('size_x')
		self.size_z = B.prop_get('size_z')

class SphereObjectParams:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, offset_x:float = 0.0, offset_y:float = 0.0, offset_z:float = 0.0, r:float = 0.0, inner_r:float = 0.0):
		self.object_type = object_type
		self.object_id = object_id
		self.offset_x = offset_x
		self.offset_y = offset_y
		self.offset_z = offset_z
		self.r = r
		self.inner_r = inner_r
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('offset_x', self.offset_x, BScmFmt.TypeDouble)
		B.prop_add('offset_y', self.offset_y, BScmFmt.TypeDouble)
		B.prop_add('offset_z', self.offset_z, BScmFmt.TypeDouble)
		B.prop_add('r', self.r, BScmFmt.TypeDouble)
		B.prop_add('inner_r', self.inner_r, BScmFmt.TypeDouble)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.offset_x = B.prop_get('offset_x')
		self.offset_y = B.prop_get('offset_y')
		self.offset_z = B.prop_get('offset_z')
		self.r = B.prop_get('r')
		self.inner_r = B.prop_get('inner_r')

class TubeObjectParams:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, offset_x:float = 0.0, offset_y:float = 0.0, offset_z:float = 0.0, r:float = 0.0, inner_r:float = 0.0, h:float = 0.0, sector_angle:float = 0.0, sector_start_angle:float = 0.0, hexahedral_mesh:bool = False, elem_count_axial:int = 0, elem_count_radial:int = 0, elem_count_tangential:int = 0):
		self.object_type = object_type
		self.object_id = object_id
		self.offset_x = offset_x
		self.offset_y = offset_y
		self.offset_z = offset_z
		self.r = r
		self.inner_r = inner_r
		self.h = h
		self.sector_angle = sector_angle
		self.sector_start_angle = sector_start_angle
		self.hexahedral_mesh = hexahedral_mesh
		self.elem_count_axial = elem_count_axial
		self.elem_count_radial = elem_count_radial
		self.elem_count_tangential = elem_count_tangential
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('offset_x', self.offset_x, BScmFmt.TypeDouble)
		B.prop_add('offset_y', self.offset_y, BScmFmt.TypeDouble)
		B.prop_add('offset_z', self.offset_z, BScmFmt.TypeDouble)
		B.prop_add('r', self.r, BScmFmt.TypeDouble)
		B.prop_add('inner_r', self.inner_r, BScmFmt.TypeDouble)
		B.prop_add('h', self.h, BScmFmt.TypeDouble)
		B.prop_add('sector_angle', self.sector_angle, BScmFmt.TypeDouble)
		B.prop_add('sector_start_angle', self.sector_start_angle, BScmFmt.TypeDouble)
		B.prop_add('hexahedral_mesh', self.hexahedral_mesh, BScmFmt.TypeBool)
		B.prop_add('elem_count_axial', self.elem_count_axial, BScmFmt.TypeInt)
		B.prop_add('elem_count_radial', self.elem_count_radial, BScmFmt.TypeInt)
		B.prop_add('elem_count_tangential', self.elem_count_tangential, BScmFmt.TypeInt)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.offset_x = B.prop_get('offset_x')
		self.offset_y = B.prop_get('offset_y')
		self.offset_z = B.prop_get('offset_z')
		self.r = B.prop_get('r')
		self.inner_r = B.prop_get('inner_r')
		self.h = B.prop_get('h')
		self.sector_angle = B.prop_get('sector_angle')
		self.sector_start_angle = B.prop_get('sector_start_angle')
		self.hexahedral_mesh = B.prop_get('hexahedral_mesh')
		self.elem_count_axial = B.prop_get('elem_count_axial')
		self.elem_count_radial = B.prop_get('elem_count_radial')
		self.elem_count_tangential = B.prop_get('elem_count_tangential')

class ExtrudedObject:
	def __init__(self, file:str = '', object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, depth:float = 0.0, mesh_adapt_koef:float = 1):
		self.file = file
		self.object_type = object_type
		self.object_id = object_id
		self.depth = depth
		self.mesh_adapt_koef = mesh_adapt_koef
	def store(self, B):
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('depth', self.depth, BScmFmt.TypeDouble)
		B.prop_add('mesh_adapt_koef', self.mesh_adapt_koef, BScmFmt.TypeDouble)
	def load(self, B):
		self.file = B.prop_get('file')
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.depth = B.prop_get('depth')
		self.mesh_adapt_koef = B.prop_get('mesh_adapt_koef')

class RevolvedObject:
	def __init__(self, file:str = '', object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, angle:float = 0.0, use_x_axis:bool = False, mesh_adapt_koef:float = 1):
		self.file = file
		self.object_type = object_type
		self.object_id = object_id
		self.angle = angle
		self.use_x_axis = use_x_axis
		self.mesh_adapt_koef = mesh_adapt_koef
	def store(self, B):
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('angle', self.angle, BScmFmt.TypeDouble)
		B.prop_add('use_x_axis', self.use_x_axis, BScmFmt.TypeBool)
		B.prop_add('mesh_adapt_koef', self.mesh_adapt_koef, BScmFmt.TypeDouble)
	def load(self, B):
		self.file = B.prop_get('file')
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.angle = B.prop_get('angle')
		self.use_x_axis = B.prop_get('use_x_axis')
		self.mesh_adapt_koef = B.prop_get('mesh_adapt_koef')

class FileObject:
	def __init__(self, file:str = '', object_type:ObjectType = ObjectType.Nothing, object_id:int = 0):
		self.file = file
		self.object_type = object_type
		self.object_id = object_id
	def store(self, B):
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
	def load(self, B):
		self.file = B.prop_get('file')
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')

class GravityPositioning:
	def __init__(self, moving_object_type:ObjectType = ObjectType.Nothing, moving_object_id:int = 0, target_object_type:ObjectType = ObjectType.Nothing, target_object_id:int = 0):
		self.moving_object_type = moving_object_type
		self.moving_object_id = moving_object_id
		self.target_object_type = target_object_type
		self.target_object_id = target_object_id
	def store(self, B):
		B.enum_add('moving_object_type', self.moving_object_type)
		B.prop_add('moving_object_id', self.moving_object_id, BScmFmt.TypeInt)
		B.enum_add('target_object_type', self.target_object_type)
		B.prop_add('target_object_id', self.target_object_id, BScmFmt.TypeInt)
	def load(self, B):
		self.moving_object_type = B.enum_get('moving_object_type', ObjectType)
		self.moving_object_id = B.prop_get('moving_object_id')
		self.target_object_type = B.enum_get('target_object_type', ObjectType)
		self.target_object_id = B.prop_get('target_object_id')

class KeyNames:
	def __init__(self, names:List[str] = []):
		self.names = names.copy()
	def store(self, B):
		B.strings_add('names', self.names)
	def load(self, B):
		self.names = B.strings_get('names')

class SendKey:
	def __init__(self, key:str = '', ctrl:bool = False, alt:bool = False, shift:bool = False):
		self.key = key
		self.ctrl = ctrl
		self.alt = alt
		self.shift = shift
	def store(self, B):
		B.prop_add('key', self.key, BScmFmt.TypeStr)
		B.prop_add('ctrl', self.ctrl, BScmFmt.TypeBool)
		B.prop_add('alt', self.alt, BScmFmt.TypeBool)
		B.prop_add('shift', self.shift, BScmFmt.TypeBool)
	def load(self, B):
		self.key = B.prop_get('key')
		self.ctrl = B.prop_get('ctrl')
		self.alt = B.prop_get('alt')
		self.shift = B.prop_get('shift')

class QFormLang:
	def __init__(self, language:Language = Language.russian):
		self.language = language
	def store(self, B):
		B.enum_add('language', self.language)
	def load(self, B):
		self.language = B.enum_get('language', Language)

class LogParams:
	def __init__(self, log_input:bool = False, log_output:bool = True):
		self.log_input = log_input
		self.log_output = log_output
	def store(self, B):
		B.prop_add('log_input', self.log_input, BScmFmt.TypeBool)
		B.prop_add('log_output', self.log_output, BScmFmt.TypeBool)
	def load(self, B):
		self.log_input = B.prop_get('log_input')
		self.log_output = B.prop_get('log_output')

class LogFile:
	def __init__(self, file:str = '', log_format:LogFormat = LogFormat.FromFileExtension):
		self.file = file
		self.log_format = log_format
	def store(self, B):
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.enum_add('log_format', self.log_format)
	def load(self, B):
		self.file = B.prop_get('file')
		self.log_format = B.enum_get('log_format', LogFormat)

class MeshApexId:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, apex:int = 0):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.apex = apex
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('apex', self.apex, BScmFmt.TypeInt)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.apex = B.prop_get('apex')

class MeshApex:
	def __init__(self, node:int = -1, edges:List['MeshApexEdge'] = []):
		self.node = node
		self.edges = edges
	def store(self, B):
		B.prop_add('node', self.node, BScmFmt.TypeInt)
		B.objects_add('edges', self.edges)
	def load(self, B):
		self.node = B.prop_get('node')
		self.edges = B.objects_get('edges', MeshApexEdge)

class MeshApexEdge:
	def __init__(self, edge:int = 0, is_start:bool = False):
		self.edge = edge
		self.is_start = is_start
	def store(self, B):
		B.prop_add('edge', self.edge, BScmFmt.TypeInt)
		B.prop_add('is_start', self.is_start, BScmFmt.TypeBool)
	def load(self, B):
		self.edge = B.prop_get('edge')
		self.is_start = B.prop_get('is_start')

class MeshObjectId:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')

class MeshCubics:
	def __init__(self, node_1:List[int] = [], node_2:List[int] = [], node_3:List[int] = [], node_4:List[int] = [], node_5:List[int] = [], node_6:List[int] = [], node_7:List[int] = [], node_8:List[int] = []):
		self.node_1 = node_1.copy()
		self.node_2 = node_2.copy()
		self.node_3 = node_3.copy()
		self.node_4 = node_4.copy()
		self.node_5 = node_5.copy()
		self.node_6 = node_6.copy()
		self.node_7 = node_7.copy()
		self.node_8 = node_8.copy()
	def store(self, B):
		B.prop_add('node_1', self.node_1, BScmFmt.TypeIntList)
		B.prop_add('node_2', self.node_2, BScmFmt.TypeIntList)
		B.prop_add('node_3', self.node_3, BScmFmt.TypeIntList)
		B.prop_add('node_4', self.node_4, BScmFmt.TypeIntList)
		B.prop_add('node_5', self.node_5, BScmFmt.TypeIntList)
		B.prop_add('node_6', self.node_6, BScmFmt.TypeIntList)
		B.prop_add('node_7', self.node_7, BScmFmt.TypeIntList)
		B.prop_add('node_8', self.node_8, BScmFmt.TypeIntList)
	def load(self, B):
		self.node_1 = B.prop_get('node_1')
		self.node_2 = B.prop_get('node_2')
		self.node_3 = B.prop_get('node_3')
		self.node_4 = B.prop_get('node_4')
		self.node_5 = B.prop_get('node_5')
		self.node_6 = B.prop_get('node_6')
		self.node_7 = B.prop_get('node_7')
		self.node_8 = B.prop_get('node_8')

class MeshEdgeId:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, edge:int = 0):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.edge = edge
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('edge', self.edge, BScmFmt.TypeInt)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.edge = B.prop_get('edge')

class MeshEdge:
	def __init__(self, nodes:List[int] = [], apex_start:int = -1, apex_end:int = -1, adjacent_face_1:int = -1, adjacent_face_2:int = -1, adjacent_face_1_bound:int = -1, adjacent_face_2_bound:int = -1):
		self.nodes = nodes.copy()
		self.apex_start = apex_start
		self.apex_end = apex_end
		self.adjacent_face_1 = adjacent_face_1
		self.adjacent_face_2 = adjacent_face_2
		self.adjacent_face_1_bound = adjacent_face_1_bound
		self.adjacent_face_2_bound = adjacent_face_2_bound
	def store(self, B):
		B.prop_add('nodes', self.nodes, BScmFmt.TypeIntList)
		B.prop_add('apex_start', self.apex_start, BScmFmt.TypeInt)
		B.prop_add('apex_end', self.apex_end, BScmFmt.TypeInt)
		B.prop_add('adjacent_face_1', self.adjacent_face_1, BScmFmt.TypeInt)
		B.prop_add('adjacent_face_2', self.adjacent_face_2, BScmFmt.TypeInt)
		B.prop_add('adjacent_face_1_bound', self.adjacent_face_1_bound, BScmFmt.TypeInt)
		B.prop_add('adjacent_face_2_bound', self.adjacent_face_2_bound, BScmFmt.TypeInt)
	def load(self, B):
		self.nodes = B.prop_get('nodes')
		self.apex_start = B.prop_get('apex_start')
		self.apex_end = B.prop_get('apex_end')
		self.adjacent_face_1 = B.prop_get('adjacent_face_1')
		self.adjacent_face_2 = B.prop_get('adjacent_face_2')
		self.adjacent_face_1_bound = B.prop_get('adjacent_face_1_bound')
		self.adjacent_face_2_bound = B.prop_get('adjacent_face_2_bound')

class MeshFaceId:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, face:int = 0):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.face = face
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('face', self.face, BScmFmt.TypeInt)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.face = B.prop_get('face')

class MeshFace:
	def __init__(self, type:FaceType = FaceType.Unspecified, triangles:List[int] = [], quadrangles:List[int] = [], inner_nodes:List[int] = [], bound_nodes:List[int] = [], bounds:List['MeshFaceBound'] = []):
		self.type = type
		self.triangles = triangles.copy()
		self.quadrangles = quadrangles.copy()
		self.inner_nodes = inner_nodes.copy()
		self.bound_nodes = bound_nodes.copy()
		self.bounds = bounds
	def store(self, B):
		B.enum_add('type', self.type)
		B.prop_add('triangles', self.triangles, BScmFmt.TypeIntList)
		B.prop_add('quadrangles', self.quadrangles, BScmFmt.TypeIntList)
		B.prop_add('inner_nodes', self.inner_nodes, BScmFmt.TypeIntList)
		B.prop_add('bound_nodes', self.bound_nodes, BScmFmt.TypeIntList)
		B.objects_add('bounds', self.bounds)
	def load(self, B):
		self.type = B.enum_get('type', FaceType)
		self.triangles = B.prop_get('triangles')
		self.quadrangles = B.prop_get('quadrangles')
		self.inner_nodes = B.prop_get('inner_nodes')
		self.bound_nodes = B.prop_get('bound_nodes')
		self.bounds = B.objects_get('bounds', MeshFaceBound)

class MeshFaceBound:
	def __init__(self, nodes:List[int] = [], edges:List[int] = [], edge_directions:List[bool] = []):
		self.nodes = nodes.copy()
		self.edges = edges.copy()
		self.edge_directions = edge_directions.copy()
	def store(self, B):
		B.prop_add('nodes', self.nodes, BScmFmt.TypeIntList)
		B.prop_add('edges', self.edges, BScmFmt.TypeIntList)
		B.prop_add('edge_directions', self.edge_directions, BScmFmt.TypeBoolList)
	def load(self, B):
		self.nodes = B.prop_get('nodes')
		self.edges = B.prop_get('edges')
		self.edge_directions = B.prop_get('edge_directions')

class FaceTypes:
	def __init__(self, type:List[FaceType] = []):
		self.type = type.copy()
	def store(self, B):
		B.enum_array_add('type', self.type)
	def load(self, B):
		self.type = B.enum_array_get('type', FaceType)

class MeshCoords:
	def __init__(self, x:List[float] = [], y:List[float] = [], z:List[float] = [], units:str = ''):
		self.x = x.copy()
		self.y = y.copy()
		self.z = z.copy()
		self.units = units
	def store(self, B):
		B.prop_add('x', self.x, BScmFmt.TypeDoubleList)
		B.prop_add('y', self.y, BScmFmt.TypeDoubleList)
		B.prop_add('z', self.z, BScmFmt.TypeDoubleList)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.x = B.prop_get('x')
		self.y = B.prop_get('y')
		self.z = B.prop_get('z')
		self.units = B.prop_get('units')

class MeshNodeOwners:
	def __init__(self, owner_type:List[MeshNodeOwnerType] = [], owner_id:List[int] = []):
		self.owner_type = owner_type.copy()
		self.owner_id = owner_id.copy()
	def store(self, B):
		B.enum_array_add('owner_type', self.owner_type)
		B.prop_add('owner_id', self.owner_id, BScmFmt.TypeIntList)
	def load(self, B):
		self.owner_type = B.enum_array_get('owner_type', MeshNodeOwnerType)
		self.owner_id = B.prop_get('owner_id')

class ObjectPoint:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0, on_surface:bool = False):
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
		self.on_surface = on_surface
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
		B.prop_add('on_surface', self.on_surface, BScmFmt.TypeBool)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.on_surface = B.prop_get('on_surface')

class MeshPoint:
	def __init__(self, x:float = 0.0, y:float = 0.0, z:float = 0.0, mesh_index:int = 0, on_surface:bool = False, node:List[int] = [], node_weight:List[float] = [], units:str = '', not_defined:bool = False):
		self.x = x
		self.y = y
		self.z = z
		self.mesh_index = mesh_index
		self.on_surface = on_surface
		self.node = node.copy()
		self.node_weight = node_weight.copy()
		self.units = units
		self.not_defined = not_defined
	def store(self, B):
		B.prop_add('x', self.x, BScmFmt.TypeDouble)
		B.prop_add('y', self.y, BScmFmt.TypeDouble)
		B.prop_add('z', self.z, BScmFmt.TypeDouble)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('on_surface', self.on_surface, BScmFmt.TypeBool)
		B.prop_add('node', self.node, BScmFmt.TypeIntList)
		B.prop_add('node_weight', self.node_weight, BScmFmt.TypeDoubleList)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
		B.prop_add('not_defined', self.not_defined, BScmFmt.TypeBool)
	def load(self, B):
		self.x = B.prop_get('x')
		self.y = B.prop_get('y')
		self.z = B.prop_get('z')
		self.mesh_index = B.prop_get('mesh_index')
		self.on_surface = B.prop_get('on_surface')
		self.node = B.prop_get('node')
		self.node_weight = B.prop_get('node_weight')
		self.units = B.prop_get('units')
		self.not_defined = B.prop_get('not_defined')

class MeshProperties:
	def __init__(self, dim:int = 0, element_count_volumetric:int = 0, element_count_surface:int = 0, node_count:int = 0, node_count_internal:int = 0, node_count_surface:int = 0, triangle_count:int = 0, triangle_quad_count:int = 0, quadrangle_count:int = 0, tetrahedron_count:int = 0, cubic_count:int = 0, face_count:int = 0, edge_count:int = 0, apex_count:int = 0):
		self.dim = dim
		self.element_count_volumetric = element_count_volumetric
		self.element_count_surface = element_count_surface
		self.node_count = node_count
		self.node_count_internal = node_count_internal
		self.node_count_surface = node_count_surface
		self.triangle_count = triangle_count
		self.triangle_quad_count = triangle_quad_count
		self.quadrangle_count = quadrangle_count
		self.tetrahedron_count = tetrahedron_count
		self.cubic_count = cubic_count
		self.face_count = face_count
		self.edge_count = edge_count
		self.apex_count = apex_count
	def store(self, B):
		B.prop_add('dim', self.dim, BScmFmt.TypeInt)
		B.prop_add('element_count_volumetric', self.element_count_volumetric, BScmFmt.TypeInt)
		B.prop_add('element_count_surface', self.element_count_surface, BScmFmt.TypeInt)
		B.prop_add('node_count', self.node_count, BScmFmt.TypeInt)
		B.prop_add('node_count_internal', self.node_count_internal, BScmFmt.TypeInt)
		B.prop_add('node_count_surface', self.node_count_surface, BScmFmt.TypeInt)
		B.prop_add('triangle_count', self.triangle_count, BScmFmt.TypeInt)
		B.prop_add('triangle_quad_count', self.triangle_quad_count, BScmFmt.TypeInt)
		B.prop_add('quadrangle_count', self.quadrangle_count, BScmFmt.TypeInt)
		B.prop_add('tetrahedron_count', self.tetrahedron_count, BScmFmt.TypeInt)
		B.prop_add('cubic_count', self.cubic_count, BScmFmt.TypeInt)
		B.prop_add('face_count', self.face_count, BScmFmt.TypeInt)
		B.prop_add('edge_count', self.edge_count, BScmFmt.TypeInt)
		B.prop_add('apex_count', self.apex_count, BScmFmt.TypeInt)
	def load(self, B):
		self.dim = B.prop_get('dim')
		self.element_count_volumetric = B.prop_get('element_count_volumetric')
		self.element_count_surface = B.prop_get('element_count_surface')
		self.node_count = B.prop_get('node_count')
		self.node_count_internal = B.prop_get('node_count_internal')
		self.node_count_surface = B.prop_get('node_count_surface')
		self.triangle_count = B.prop_get('triangle_count')
		self.triangle_quad_count = B.prop_get('triangle_quad_count')
		self.quadrangle_count = B.prop_get('quadrangle_count')
		self.tetrahedron_count = B.prop_get('tetrahedron_count')
		self.cubic_count = B.prop_get('cubic_count')
		self.face_count = B.prop_get('face_count')
		self.edge_count = B.prop_get('edge_count')
		self.apex_count = B.prop_get('apex_count')

class MeshQuadrangles:
	def __init__(self, node_1:List[int] = [], node_2:List[int] = [], node_3:List[int] = [], node_4:List[int] = []):
		self.node_1 = node_1.copy()
		self.node_2 = node_2.copy()
		self.node_3 = node_3.copy()
		self.node_4 = node_4.copy()
	def store(self, B):
		B.prop_add('node_1', self.node_1, BScmFmt.TypeIntList)
		B.prop_add('node_2', self.node_2, BScmFmt.TypeIntList)
		B.prop_add('node_3', self.node_3, BScmFmt.TypeIntList)
		B.prop_add('node_4', self.node_4, BScmFmt.TypeIntList)
	def load(self, B):
		self.node_1 = B.prop_get('node_1')
		self.node_2 = B.prop_get('node_2')
		self.node_3 = B.prop_get('node_3')
		self.node_4 = B.prop_get('node_4')

class MeshTetrahedrons:
	def __init__(self, node_1:List[int] = [], node_2:List[int] = [], node_3:List[int] = [], node_4:List[int] = []):
		self.node_1 = node_1.copy()
		self.node_2 = node_2.copy()
		self.node_3 = node_3.copy()
		self.node_4 = node_4.copy()
	def store(self, B):
		B.prop_add('node_1', self.node_1, BScmFmt.TypeIntList)
		B.prop_add('node_2', self.node_2, BScmFmt.TypeIntList)
		B.prop_add('node_3', self.node_3, BScmFmt.TypeIntList)
		B.prop_add('node_4', self.node_4, BScmFmt.TypeIntList)
	def load(self, B):
		self.node_1 = B.prop_get('node_1')
		self.node_2 = B.prop_get('node_2')
		self.node_3 = B.prop_get('node_3')
		self.node_4 = B.prop_get('node_4')

class MeshTriangles:
	def __init__(self, node_1:List[int] = [], node_2:List[int] = [], node_3:List[int] = []):
		self.node_1 = node_1.copy()
		self.node_2 = node_2.copy()
		self.node_3 = node_3.copy()
	def store(self, B):
		B.prop_add('node_1', self.node_1, BScmFmt.TypeIntList)
		B.prop_add('node_2', self.node_2, BScmFmt.TypeIntList)
		B.prop_add('node_3', self.node_3, BScmFmt.TypeIntList)
	def load(self, B):
		self.node_1 = B.prop_get('node_1')
		self.node_2 = B.prop_get('node_2')
		self.node_3 = B.prop_get('node_3')

class MouseClick:
	def __init__(self, button:MouseButton = MouseButton.Right, use_current_pos:bool = True, coord_x:int = 0, coord_y:int = 0, ctrl:bool = False, alt:bool = False, shift:bool = False):
		self.button = button
		self.use_current_pos = use_current_pos
		self.coord_x = coord_x
		self.coord_y = coord_y
		self.ctrl = ctrl
		self.alt = alt
		self.shift = shift
	def store(self, B):
		B.enum_add('button', self.button)
		B.prop_add('use_current_pos', self.use_current_pos, BScmFmt.TypeBool)
		B.prop_add('coord_x', self.coord_x, BScmFmt.TypeInt)
		B.prop_add('coord_y', self.coord_y, BScmFmt.TypeInt)
		B.prop_add('ctrl', self.ctrl, BScmFmt.TypeBool)
		B.prop_add('alt', self.alt, BScmFmt.TypeBool)
		B.prop_add('shift', self.shift, BScmFmt.TypeBool)
	def load(self, B):
		self.button = B.enum_get('button', MouseButton)
		self.use_current_pos = B.prop_get('use_current_pos')
		self.coord_x = B.prop_get('coord_x')
		self.coord_y = B.prop_get('coord_y')
		self.ctrl = B.prop_get('ctrl')
		self.alt = B.prop_get('alt')
		self.shift = B.prop_get('shift')

class MousePos:
	def __init__(self, coord_x:int = 0, coord_y:int = 0):
		self.coord_x = coord_x
		self.coord_y = coord_y
	def store(self, B):
		B.prop_add('coord_x', self.coord_x, BScmFmt.TypeInt)
		B.prop_add('coord_y', self.coord_y, BScmFmt.TypeInt)
	def load(self, B):
		self.coord_x = B.prop_get('coord_x')
		self.coord_y = B.prop_get('coord_y')

class MsgBox:
	def __init__(self, msg:str = '', button_ok:bool = True, button_cancel:bool = False, button_yes:bool = False, button_no:bool = False, button_retry:bool = False, button_continue:bool = False, button_close:bool = False, button_save_as:bool = False):
		self.msg = msg
		self.button_ok = button_ok
		self.button_cancel = button_cancel
		self.button_yes = button_yes
		self.button_no = button_no
		self.button_retry = button_retry
		self.button_continue = button_continue
		self.button_close = button_close
		self.button_save_as = button_save_as
	def store(self, B):
		B.prop_add('msg', self.msg, BScmFmt.TypeStr)
		B.prop_add('button_ok', self.button_ok, BScmFmt.TypeBool)
		B.prop_add('button_cancel', self.button_cancel, BScmFmt.TypeBool)
		B.prop_add('button_yes', self.button_yes, BScmFmt.TypeBool)
		B.prop_add('button_no', self.button_no, BScmFmt.TypeBool)
		B.prop_add('button_retry', self.button_retry, BScmFmt.TypeBool)
		B.prop_add('button_continue', self.button_continue, BScmFmt.TypeBool)
		B.prop_add('button_close', self.button_close, BScmFmt.TypeBool)
		B.prop_add('button_save_as', self.button_save_as, BScmFmt.TypeBool)
	def load(self, B):
		self.msg = B.prop_get('msg')
		self.button_ok = B.prop_get('button_ok')
		self.button_cancel = B.prop_get('button_cancel')
		self.button_yes = B.prop_get('button_yes')
		self.button_no = B.prop_get('button_no')
		self.button_retry = B.prop_get('button_retry')
		self.button_continue = B.prop_get('button_continue')
		self.button_close = B.prop_get('button_close')
		self.button_save_as = B.prop_get('button_save_as')

class PressedDialogButton:
	def __init__(self, button:DialogButton = DialogButton.Ok):
		self.button = button
	def store(self, B):
		B.enum_add('button', self.button)
	def load(self, B):
		self.button = B.enum_get('button', DialogButton)

class ObjectTransform:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, matrix4x4:List[float] = []):
		self.object_type = object_type
		self.object_id = object_id
		self.matrix4x4 = matrix4x4.copy()
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('matrix4x4', self.matrix4x4, BScmFmt.TypeDoubleList)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.matrix4x4 = B.prop_get('matrix4x4')

class ObjectContact:
	def __init__(self, moving_object_type:ObjectType = ObjectType.Nothing, moving_object_id:int = 0, target_object_type:ObjectType = ObjectType.Nothing, target_object_id:int = 0, direction:Direction = Direction.X, reverse_direction:bool = False, move_dependent_objects:bool = False, move_additional_objects:List['ObjectId'] = []):
		self.moving_object_type = moving_object_type
		self.moving_object_id = moving_object_id
		self.target_object_type = target_object_type
		self.target_object_id = target_object_id
		self.direction = direction
		self.reverse_direction = reverse_direction
		self.move_dependent_objects = move_dependent_objects
		self.move_additional_objects = move_additional_objects
	def store(self, B):
		B.enum_add('moving_object_type', self.moving_object_type)
		B.prop_add('moving_object_id', self.moving_object_id, BScmFmt.TypeInt)
		B.enum_add('target_object_type', self.target_object_type)
		B.prop_add('target_object_id', self.target_object_id, BScmFmt.TypeInt)
		B.enum_add('direction', self.direction)
		B.prop_add('reverse_direction', self.reverse_direction, BScmFmt.TypeBool)
		B.prop_add('move_dependent_objects', self.move_dependent_objects, BScmFmt.TypeBool)
		B.objects_add('move_additional_objects', self.move_additional_objects)
	def load(self, B):
		self.moving_object_type = B.enum_get('moving_object_type', ObjectType)
		self.moving_object_id = B.prop_get('moving_object_id')
		self.target_object_type = B.enum_get('target_object_type', ObjectType)
		self.target_object_id = B.prop_get('target_object_id')
		self.direction = B.enum_get('direction', Direction)
		self.reverse_direction = B.prop_get('reverse_direction')
		self.move_dependent_objects = B.prop_get('move_dependent_objects')
		self.move_additional_objects = B.objects_get('move_additional_objects', ObjectId)

class ObjectConvert:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, new_type:ObjectType = ObjectType.Nothing, new_id:int = 0):
		self.object_type = object_type
		self.object_id = object_id
		self.new_type = new_type
		self.new_id = new_id
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.enum_add('new_type', self.new_type)
		B.prop_add('new_id', self.new_id, BScmFmt.TypeInt)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.new_type = B.enum_get('new_type', ObjectType)
		self.new_id = B.prop_get('new_id')

class DisplayMode:
	def __init__(self, type:ObjectType = ObjectType.Nothing, id:int = 0, mode:DisplayModes = DisplayModes.visible, value:bool = True):
		self.type = type
		self.id = id
		self.mode = mode
		self.value = value
	def store(self, B):
		B.enum_add('type', self.type)
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('mode', self.mode)
		B.prop_add('value', self.value, BScmFmt.TypeBool)
	def load(self, B):
		self.type = B.enum_get('type', ObjectType)
		self.id = B.prop_get('id')
		self.mode = B.enum_get('mode', DisplayModes)
		self.value = B.prop_get('value')

class ObjectName:
	def __init__(self, name:str = ''):
		self.name = name
	def store(self, B):
		B.prop_add('name', self.name, BScmFmt.TypeStr)
	def load(self, B):
		self.name = B.prop_get('name')

class FindByColor:
	def __init__(self, color_R:int = 0, color_G:int = 0, color_B:int = 0, pick_by_body_color:bool = False, pick_by_face_color:bool = False):
		self.color_R = color_R
		self.color_G = color_G
		self.color_B = color_B
		self.pick_by_body_color = pick_by_body_color
		self.pick_by_face_color = pick_by_face_color
	def store(self, B):
		B.prop_add('color_R', self.color_R, BScmFmt.TypeInt)
		B.prop_add('color_G', self.color_G, BScmFmt.TypeInt)
		B.prop_add('color_B', self.color_B, BScmFmt.TypeInt)
		B.prop_add('pick_by_body_color', self.pick_by_body_color, BScmFmt.TypeBool)
		B.prop_add('pick_by_face_color', self.pick_by_face_color, BScmFmt.TypeBool)
	def load(self, B):
		self.color_R = B.prop_get('color_R')
		self.color_G = B.prop_get('color_G')
		self.color_B = B.prop_get('color_B')
		self.pick_by_body_color = B.prop_get('pick_by_body_color')
		self.pick_by_face_color = B.prop_get('pick_by_face_color')

class FindByPoint:
	def __init__(self, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0):
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
	def store(self, B):
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
	def load(self, B):
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')

class ObjectMove:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, vector_x:float = 0.0, vector_y:float = 0.0, vector_z:float = 0.0, move_dependent_objects:bool = False):
		self.object_type = object_type
		self.object_id = object_id
		self.vector_x = vector_x
		self.vector_y = vector_y
		self.vector_z = vector_z
		self.move_dependent_objects = move_dependent_objects
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('vector_x', self.vector_x, BScmFmt.TypeDouble)
		B.prop_add('vector_y', self.vector_y, BScmFmt.TypeDouble)
		B.prop_add('vector_z', self.vector_z, BScmFmt.TypeDouble)
		B.prop_add('move_dependent_objects', self.move_dependent_objects, BScmFmt.TypeBool)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.vector_x = B.prop_get('vector_x')
		self.vector_y = B.prop_get('vector_y')
		self.vector_z = B.prop_get('vector_z')
		self.move_dependent_objects = B.prop_get('move_dependent_objects')

class ObjectMoveAxis:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, axis:int = 0, distance:float = 0.0, move_dependent_objects:bool = False):
		self.object_type = object_type
		self.object_id = object_id
		self.axis = axis
		self.distance = distance
		self.move_dependent_objects = move_dependent_objects
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('axis', self.axis, BScmFmt.TypeInt)
		B.prop_add('distance', self.distance, BScmFmt.TypeDouble)
		B.prop_add('move_dependent_objects', self.move_dependent_objects, BScmFmt.TypeBool)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.axis = B.prop_get('axis')
		self.distance = B.prop_get('distance')
		self.move_dependent_objects = B.prop_get('move_dependent_objects')

class ObjectRotate:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0, vector_x:float = 0.0, vector_y:float = 0.0, vector_z:float = 0.0, angle:float = 0.0, rotate_dependent_objects:bool = False):
		self.object_type = object_type
		self.object_id = object_id
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
		self.vector_x = vector_x
		self.vector_y = vector_y
		self.vector_z = vector_z
		self.angle = angle
		self.rotate_dependent_objects = rotate_dependent_objects
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
		B.prop_add('vector_x', self.vector_x, BScmFmt.TypeDouble)
		B.prop_add('vector_y', self.vector_y, BScmFmt.TypeDouble)
		B.prop_add('vector_z', self.vector_z, BScmFmt.TypeDouble)
		B.prop_add('angle', self.angle, BScmFmt.TypeDouble)
		B.prop_add('rotate_dependent_objects', self.rotate_dependent_objects, BScmFmt.TypeBool)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.vector_x = B.prop_get('vector_x')
		self.vector_y = B.prop_get('vector_y')
		self.vector_z = B.prop_get('vector_z')
		self.angle = B.prop_get('angle')
		self.rotate_dependent_objects = B.prop_get('rotate_dependent_objects')

class ObjectRotateAxis:
	def __init__(self, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, axis:int = 0, angle:float = 0.0, rotate_dependent_objects:bool = False):
		self.object_type = object_type
		self.object_id = object_id
		self.axis = axis
		self.angle = angle
		self.rotate_dependent_objects = rotate_dependent_objects
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('axis', self.axis, BScmFmt.TypeInt)
		B.prop_add('angle', self.angle, BScmFmt.TypeDouble)
		B.prop_add('rotate_dependent_objects', self.rotate_dependent_objects, BScmFmt.TypeBool)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.axis = B.prop_get('axis')
		self.angle = B.prop_get('angle')
		self.rotate_dependent_objects = B.prop_get('rotate_dependent_objects')

class TypeSetByColor:
	def __init__(self, type:ObjectType = ObjectType.Nothing, id:int = 0, color_R:int = 0, color_G:int = 0, color_B:int = 0, pick_by_body_color:bool = False, pick_by_face_color:bool = False):
		self.type = type
		self.id = id
		self.color_R = color_R
		self.color_G = color_G
		self.color_B = color_B
		self.pick_by_body_color = pick_by_body_color
		self.pick_by_face_color = pick_by_face_color
	def store(self, B):
		B.enum_add('type', self.type)
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('color_R', self.color_R, BScmFmt.TypeInt)
		B.prop_add('color_G', self.color_G, BScmFmt.TypeInt)
		B.prop_add('color_B', self.color_B, BScmFmt.TypeInt)
		B.prop_add('pick_by_body_color', self.pick_by_body_color, BScmFmt.TypeBool)
		B.prop_add('pick_by_face_color', self.pick_by_face_color, BScmFmt.TypeBool)
	def load(self, B):
		self.type = B.enum_get('type', ObjectType)
		self.id = B.prop_get('id')
		self.color_R = B.prop_get('color_R')
		self.color_G = B.prop_get('color_G')
		self.color_B = B.prop_get('color_B')
		self.pick_by_body_color = B.prop_get('pick_by_body_color')
		self.pick_by_face_color = B.prop_get('pick_by_face_color')

class TypeSetByPoint:
	def __init__(self, type:ObjectType = ObjectType.Nothing, id:int = 0, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0):
		self.type = type
		self.id = id
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
	def store(self, B):
		B.enum_add('type', self.type)
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
	def load(self, B):
		self.type = B.enum_get('type', ObjectType)
		self.id = B.prop_get('id')
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')

class ObjectsInDirection:
	def __init__(self, vector_x:float = 0.0, vector_y:float = 0.0, vector_z:float = 0.0, pick_specified_type:bool = True, pick_type:ObjectType = ObjectType.ImportedObject, objects:List['ObjectId'] = []):
		self.vector_x = vector_x
		self.vector_y = vector_y
		self.vector_z = vector_z
		self.pick_specified_type = pick_specified_type
		self.pick_type = pick_type
		self.objects = objects
	def store(self, B):
		B.prop_add('vector_x', self.vector_x, BScmFmt.TypeDouble)
		B.prop_add('vector_y', self.vector_y, BScmFmt.TypeDouble)
		B.prop_add('vector_z', self.vector_z, BScmFmt.TypeDouble)
		B.prop_add('pick_specified_type', self.pick_specified_type, BScmFmt.TypeBool)
		B.enum_add('pick_type', self.pick_type)
		B.objects_add('objects', self.objects)
	def load(self, B):
		self.vector_x = B.prop_get('vector_x')
		self.vector_y = B.prop_get('vector_y')
		self.vector_z = B.prop_get('vector_z')
		self.pick_specified_type = B.prop_get('pick_specified_type')
		self.pick_type = B.enum_get('pick_type', ObjectType)
		self.objects = B.objects_get('objects', ObjectId)

class PickDirection:
	def __init__(self, vector_x:float = 0.0, vector_y:float = 0.0, vector_z:float = 0.0, pick_specified_type:bool = True, pick_type:ObjectType = ObjectType.ImportedObject):
		self.vector_x = vector_x
		self.vector_y = vector_y
		self.vector_z = vector_z
		self.pick_specified_type = pick_specified_type
		self.pick_type = pick_type
	def store(self, B):
		B.prop_add('vector_x', self.vector_x, BScmFmt.TypeDouble)
		B.prop_add('vector_y', self.vector_y, BScmFmt.TypeDouble)
		B.prop_add('vector_z', self.vector_z, BScmFmt.TypeDouble)
		B.prop_add('pick_specified_type', self.pick_specified_type, BScmFmt.TypeBool)
		B.enum_add('pick_type', self.pick_type)
	def load(self, B):
		self.vector_x = B.prop_get('vector_x')
		self.vector_y = B.prop_get('vector_y')
		self.vector_z = B.prop_get('vector_z')
		self.pick_specified_type = B.prop_get('pick_specified_type')
		self.pick_type = B.enum_get('pick_type', ObjectType)

class OptionalItemId:
	def __init__(self, id:int = -1):
		self.id = id
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')

class OperationChecks:
	def __init__(self, errors:List[str] = [], warnings:List[str] = []):
		self.errors = errors.copy()
		self.warnings = warnings.copy()
	def store(self, B):
		B.strings_add('errors', self.errors)
		B.strings_add('warnings', self.warnings)
	def load(self, B):
		self.errors = B.strings_get('errors')
		self.warnings = B.strings_get('warnings')

class OperationCopy:
	def __init__(self, id:int = -1, source:int = 0, name:str = '', process_name:str = '', make_copy_active:bool = False):
		self.id = id
		self.source = source
		self.name = name
		self.process_name = process_name
		self.make_copy_active = make_copy_active
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('source', self.source, BScmFmt.TypeInt)
		B.prop_add('name', self.name, BScmFmt.TypeStr)
		B.prop_add('process_name', self.process_name, BScmFmt.TypeStr)
		B.prop_add('make_copy_active', self.make_copy_active, BScmFmt.TypeBool)
	def load(self, B):
		self.id = B.prop_get('id')
		self.source = B.prop_get('source')
		self.name = B.prop_get('name')
		self.process_name = B.prop_get('process_name')
		self.make_copy_active = B.prop_get('make_copy_active')

class OperationCopyFromParent:
	def __init__(self, id:int = -1, copy_bound_conds:bool = False, copy_tools:bool = False, copy_workpiece:bool = False, inherit_results:bool = False):
		self.id = id
		self.copy_bound_conds = copy_bound_conds
		self.copy_tools = copy_tools
		self.copy_workpiece = copy_workpiece
		self.inherit_results = inherit_results
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('copy_bound_conds', self.copy_bound_conds, BScmFmt.TypeBool)
		B.prop_add('copy_tools', self.copy_tools, BScmFmt.TypeBool)
		B.prop_add('copy_workpiece', self.copy_workpiece, BScmFmt.TypeBool)
		B.prop_add('inherit_results', self.inherit_results, BScmFmt.TypeBool)
	def load(self, B):
		self.id = B.prop_get('id')
		self.copy_bound_conds = B.prop_get('copy_bound_conds')
		self.copy_tools = B.prop_get('copy_tools')
		self.copy_workpiece = B.prop_get('copy_workpiece')
		self.inherit_results = B.prop_get('inherit_results')

class OperationParams:
	def __init__(self, id:int = -1, name:str = '', parent:int = 0, creation_mode:OperationCreationMode = OperationCreationMode.CreateAsNewProcess, process_name:str = ''):
		self.id = id
		self.name = name
		self.parent = parent
		self.creation_mode = creation_mode
		self.process_name = process_name
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('name', self.name, BScmFmt.TypeStr)
		B.prop_add('parent', self.parent, BScmFmt.TypeInt)
		B.enum_add('creation_mode', self.creation_mode)
		B.prop_add('process_name', self.process_name, BScmFmt.TypeStr)
	def load(self, B):
		self.id = B.prop_get('id')
		self.name = B.prop_get('name')
		self.parent = B.prop_get('parent')
		self.creation_mode = B.enum_get('creation_mode', OperationCreationMode)
		self.process_name = B.prop_get('process_name')

class Operation:
	def __init__(self, id:int = 0, type:OperationType = OperationType.none, name:str = '', dsc:str = '', childs:List['Operation'] = []):
		self.id = id
		self.type = type
		self.name = name
		self.dsc = dsc
		self.childs = childs
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('type', self.type)
		B.prop_add('name', self.name, BScmFmt.TypeStr)
		B.prop_add('dsc', self.dsc, BScmFmt.TypeStr)
		B.objects_add('childs', self.childs)
	def load(self, B):
		self.id = B.prop_get('id')
		self.type = B.enum_get('type', OperationType)
		self.name = B.prop_get('name')
		self.dsc = B.prop_get('dsc')
		self.childs = B.objects_get('childs', Operation)

class OperationGraph:
	def __init__(self, parent_defined:bool = False, parent:int = 0, childs:List[int] = []):
		self.parent_defined = parent_defined
		self.parent = parent
		self.childs = childs.copy()
	def store(self, B):
		B.prop_add('parent_defined', self.parent_defined, BScmFmt.TypeBool)
		B.prop_add('parent', self.parent, BScmFmt.TypeInt)
		B.prop_add('childs', self.childs, BScmFmt.TypeIntList)
	def load(self, B):
		self.parent_defined = B.prop_get('parent_defined')
		self.parent = B.prop_get('parent')
		self.childs = B.prop_get('childs')

class OperationInsert:
	def __init__(self, id:int = -1, name:str = '', childs:List[int] = []):
		self.id = id
		self.name = name
		self.childs = childs.copy()
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('name', self.name, BScmFmt.TypeStr)
		B.prop_add('childs', self.childs, BScmFmt.TypeIntList)
	def load(self, B):
		self.id = B.prop_get('id')
		self.name = B.prop_get('name')
		self.childs = B.prop_get('childs')

class PanelPositions:
	def __init__(self, frame_x:int = 0, frame_y:int = 0, view_x:int = 0, view_y:int = 0, view_width:int = 0, view_height:int = 0, left_panel_x:int = 0, left_panel_y:int = 0, left_panel_width:int = 0, left_panel_height:int = 0, right_panel_x:int = 0, right_panel_y:int = 0, right_panel_width:int = 0, right_panel_height:int = 0, bottom_panel_x:int = 0, bottom_panel_y:int = 0, bottom_panel_width:int = 0, bottom_panel_height:int = 0):
		self.frame_x = frame_x
		self.frame_y = frame_y
		self.view_x = view_x
		self.view_y = view_y
		self.view_width = view_width
		self.view_height = view_height
		self.left_panel_x = left_panel_x
		self.left_panel_y = left_panel_y
		self.left_panel_width = left_panel_width
		self.left_panel_height = left_panel_height
		self.right_panel_x = right_panel_x
		self.right_panel_y = right_panel_y
		self.right_panel_width = right_panel_width
		self.right_panel_height = right_panel_height
		self.bottom_panel_x = bottom_panel_x
		self.bottom_panel_y = bottom_panel_y
		self.bottom_panel_width = bottom_panel_width
		self.bottom_panel_height = bottom_panel_height
	def store(self, B):
		B.prop_add('frame_x', self.frame_x, BScmFmt.TypeInt)
		B.prop_add('frame_y', self.frame_y, BScmFmt.TypeInt)
		B.prop_add('view_x', self.view_x, BScmFmt.TypeInt)
		B.prop_add('view_y', self.view_y, BScmFmt.TypeInt)
		B.prop_add('view_width', self.view_width, BScmFmt.TypeInt)
		B.prop_add('view_height', self.view_height, BScmFmt.TypeInt)
		B.prop_add('left_panel_x', self.left_panel_x, BScmFmt.TypeInt)
		B.prop_add('left_panel_y', self.left_panel_y, BScmFmt.TypeInt)
		B.prop_add('left_panel_width', self.left_panel_width, BScmFmt.TypeInt)
		B.prop_add('left_panel_height', self.left_panel_height, BScmFmt.TypeInt)
		B.prop_add('right_panel_x', self.right_panel_x, BScmFmt.TypeInt)
		B.prop_add('right_panel_y', self.right_panel_y, BScmFmt.TypeInt)
		B.prop_add('right_panel_width', self.right_panel_width, BScmFmt.TypeInt)
		B.prop_add('right_panel_height', self.right_panel_height, BScmFmt.TypeInt)
		B.prop_add('bottom_panel_x', self.bottom_panel_x, BScmFmt.TypeInt)
		B.prop_add('bottom_panel_y', self.bottom_panel_y, BScmFmt.TypeInt)
		B.prop_add('bottom_panel_width', self.bottom_panel_width, BScmFmt.TypeInt)
		B.prop_add('bottom_panel_height', self.bottom_panel_height, BScmFmt.TypeInt)
	def load(self, B):
		self.frame_x = B.prop_get('frame_x')
		self.frame_y = B.prop_get('frame_y')
		self.view_x = B.prop_get('view_x')
		self.view_y = B.prop_get('view_y')
		self.view_width = B.prop_get('view_width')
		self.view_height = B.prop_get('view_height')
		self.left_panel_x = B.prop_get('left_panel_x')
		self.left_panel_y = B.prop_get('left_panel_y')
		self.left_panel_width = B.prop_get('left_panel_width')
		self.left_panel_height = B.prop_get('left_panel_height')
		self.right_panel_x = B.prop_get('right_panel_x')
		self.right_panel_y = B.prop_get('right_panel_y')
		self.right_panel_width = B.prop_get('right_panel_width')
		self.right_panel_height = B.prop_get('right_panel_height')
		self.bottom_panel_x = B.prop_get('bottom_panel_x')
		self.bottom_panel_y = B.prop_get('bottom_panel_y')
		self.bottom_panel_width = B.prop_get('bottom_panel_width')
		self.bottom_panel_height = B.prop_get('bottom_panel_height')

class PanelSizes:
	def __init__(self, view_height:int = 0, view_width:int = 0, left_panel_width:int = 0, right_panel_width:int = 0, bottom_panel_height:int = 0):
		self.view_height = view_height
		self.view_width = view_width
		self.left_panel_width = left_panel_width
		self.right_panel_width = right_panel_width
		self.bottom_panel_height = bottom_panel_height
	def store(self, B):
		B.prop_add('view_height', self.view_height, BScmFmt.TypeInt)
		B.prop_add('view_width', self.view_width, BScmFmt.TypeInt)
		B.prop_add('left_panel_width', self.left_panel_width, BScmFmt.TypeInt)
		B.prop_add('right_panel_width', self.right_panel_width, BScmFmt.TypeInt)
		B.prop_add('bottom_panel_height', self.bottom_panel_height, BScmFmt.TypeInt)
	def load(self, B):
		self.view_height = B.prop_get('view_height')
		self.view_width = B.prop_get('view_width')
		self.left_panel_width = B.prop_get('left_panel_width')
		self.right_panel_width = B.prop_get('right_panel_width')
		self.bottom_panel_height = B.prop_get('bottom_panel_height')

class TraceMsg:
	def __init__(self, msg:str = '', type:MessageType = MessageType.info):
		self.msg = msg
		self.type = type
	def store(self, B):
		B.prop_add('msg', self.msg, BScmFmt.TypeStr)
		B.enum_add('type', self.type)
	def load(self, B):
		self.msg = B.prop_get('msg')
		self.type = B.enum_get('type', MessageType)

class ProjectOpenAsCopy:
	def __init__(self, source_path:str = '', target_path:str = '', copy_simulation_results:bool = False):
		self.source_path = source_path
		self.target_path = target_path
		self.copy_simulation_results = copy_simulation_results
	def store(self, B):
		B.prop_add('source_path', self.source_path, BScmFmt.TypeStr)
		B.prop_add('target_path', self.target_path, BScmFmt.TypeStr)
		B.prop_add('copy_simulation_results', self.copy_simulation_results, BScmFmt.TypeBool)
	def load(self, B):
		self.source_path = B.prop_get('source_path')
		self.target_path = B.prop_get('target_path')
		self.copy_simulation_results = B.prop_get('copy_simulation_results')

class Property:
	def __init__(self, object_type:ObjectType = ObjectType.Operation, object_id:int = 0, path:str = '', property_type:PropertyType = PropertyType.Value, value:str = ''):
		self.object_type = object_type
		self.object_id = object_id
		self.path = path
		self.property_type = property_type
		self.value = value
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('path', self.path, BScmFmt.TypeStr)
		B.enum_add('property_type', self.property_type)
		B.prop_add('value', self.value, BScmFmt.TypeStr)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.path = B.prop_get('path')
		self.property_type = B.enum_get('property_type', PropertyType)
		self.value = B.prop_get('value')

class PropertyPath:
	def __init__(self, object_type:ObjectType = ObjectType.Operation, object_id:int = 0, path:str = ''):
		self.object_type = object_type
		self.object_id = object_id
		self.path = path
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('path', self.path, BScmFmt.TypeStr)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.path = B.prop_get('path')

class ArrayOfReal:
	def __init__(self, values:List[float] = [], units:str = ''):
		self.values = values.copy()
		self.units = units
	def store(self, B):
		B.prop_add('values', self.values, BScmFmt.TypeDoubleList)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.values = B.prop_get('values')
		self.units = B.prop_get('units')

class PropertyArrayOfReal:
	def __init__(self, object_type:ObjectType = ObjectType.Operation, object_id:int = 0, path:str = '', values:List[float] = []):
		self.object_type = object_type
		self.object_id = object_id
		self.path = path
		self.values = values.copy()
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('path', self.path, BScmFmt.TypeStr)
		B.prop_add('values', self.values, BScmFmt.TypeDoubleList)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.path = B.prop_get('path')
		self.values = B.prop_get('values')

class ObjectIdProperty:
	def __init__(self, object_type:ObjectType = ObjectType.Operation, object_id:int = 0, path:str = '', value_type:ObjectType = ObjectType.Nothing, value_id:int = 0):
		self.object_type = object_type
		self.object_id = object_id
		self.path = path
		self.value_type = value_type
		self.value_id = value_id
	def store(self, B):
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('path', self.path, BScmFmt.TypeStr)
		B.enum_add('value_type', self.value_type)
		B.prop_add('value_id', self.value_id, BScmFmt.TypeInt)
	def load(self, B):
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.path = B.prop_get('path')
		self.value_type = B.enum_get('value_type', ObjectType)
		self.value_id = B.prop_get('value_id')

class SessionId:
	def __init__(self, session_id:int = 0):
		self.session_id = session_id
	def store(self, B):
		B.prop_add('session_id', self.session_id, BScmFmt.TypeInt)
	def load(self, B):
		self.session_id = B.prop_get('session_id')

class ProcessId:
	def __init__(self, pid:int = 0):
		self.pid = pid
	def store(self, B):
		B.prop_add('pid', self.pid, BScmFmt.TypeInt)
	def load(self, B):
		self.pid = B.prop_get('pid')

class QFormVer:
	def __init__(self, v1:int = 0, v2:int = 0, v3:int = 0, v4:int = 0, is_cloud:bool = False, is_viewer:bool = False, configuration:str = ''):
		self.v1 = v1
		self.v2 = v2
		self.v3 = v3
		self.v4 = v4
		self.is_cloud = is_cloud
		self.is_viewer = is_viewer
		self.configuration = configuration
	def store(self, B):
		B.prop_add('v1', self.v1, BScmFmt.TypeInt)
		B.prop_add('v2', self.v2, BScmFmt.TypeInt)
		B.prop_add('v3', self.v3, BScmFmt.TypeInt)
		B.prop_add('v4', self.v4, BScmFmt.TypeInt)
		B.prop_add('is_cloud', self.is_cloud, BScmFmt.TypeBool)
		B.prop_add('is_viewer', self.is_viewer, BScmFmt.TypeBool)
		B.prop_add('configuration', self.configuration, BScmFmt.TypeStr)
	def load(self, B):
		self.v1 = B.prop_get('v1')
		self.v2 = B.prop_get('v2')
		self.v3 = B.prop_get('v3')
		self.v4 = B.prop_get('v4')
		self.is_cloud = B.prop_get('is_cloud')
		self.is_viewer = B.prop_get('is_viewer')
		self.configuration = B.prop_get('configuration')

class WindowId:
	def __init__(self, hwnd:int = 0):
		self.hwnd = hwnd
	def store(self, B):
		B.prop_add('hwnd', self.hwnd, BScmFmt.TypeInt)
	def load(self, B):
		self.hwnd = B.prop_get('hwnd')

class WindowPosition:
	def __init__(self, x:int = 0, y:int = 0, width:int = 0, height:int = 0, maximized:bool = False):
		self.x = x
		self.y = y
		self.width = width
		self.height = height
		self.maximized = maximized
	def store(self, B):
		B.prop_add('x', self.x, BScmFmt.TypeInt)
		B.prop_add('y', self.y, BScmFmt.TypeInt)
		B.prop_add('width', self.width, BScmFmt.TypeInt)
		B.prop_add('height', self.height, BScmFmt.TypeInt)
		B.prop_add('maximized', self.maximized, BScmFmt.TypeBool)
	def load(self, B):
		self.x = B.prop_get('x')
		self.y = B.prop_get('y')
		self.width = B.prop_get('width')
		self.height = B.prop_get('height')
		self.maximized = B.prop_get('maximized')

class Record:
	def __init__(self, record:float = 0.0):
		self.record = record
	def store(self, B):
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
	def load(self, B):
		self.record = B.prop_get('record')

class SectionMeshPlane:
	def __init__(self, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0, normal_x:float = 0.0, normal_y:float = 0.0, normal_z:float = 0.0, defined:bool = False, units:str = '', object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, mesh_index:int = 0, u_vector_defined:bool = False, u_vector_x:float = 0.0, u_vector_y:float = 0.0, u_vector_z:float = 0.0):
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
		self.normal_x = normal_x
		self.normal_y = normal_y
		self.normal_z = normal_z
		self.defined = defined
		self.units = units
		self.object_type = object_type
		self.object_id = object_id
		self.mesh_index = mesh_index
		self.u_vector_defined = u_vector_defined
		self.u_vector_x = u_vector_x
		self.u_vector_y = u_vector_y
		self.u_vector_z = u_vector_z
	def store(self, B):
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
		B.prop_add('normal_x', self.normal_x, BScmFmt.TypeDouble)
		B.prop_add('normal_y', self.normal_y, BScmFmt.TypeDouble)
		B.prop_add('normal_z', self.normal_z, BScmFmt.TypeDouble)
		B.prop_add('defined', self.defined, BScmFmt.TypeBool)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
		B.prop_add('u_vector_defined', self.u_vector_defined, BScmFmt.TypeBool)
		B.prop_add('u_vector_x', self.u_vector_x, BScmFmt.TypeDouble)
		B.prop_add('u_vector_y', self.u_vector_y, BScmFmt.TypeDouble)
		B.prop_add('u_vector_z', self.u_vector_z, BScmFmt.TypeDouble)
	def load(self, B):
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.normal_x = B.prop_get('normal_x')
		self.normal_y = B.prop_get('normal_y')
		self.normal_z = B.prop_get('normal_z')
		self.defined = B.prop_get('defined')
		self.units = B.prop_get('units')
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.mesh_index = B.prop_get('mesh_index')
		self.u_vector_defined = B.prop_get('u_vector_defined')
		self.u_vector_x = B.prop_get('u_vector_x')
		self.u_vector_y = B.prop_get('u_vector_y')
		self.u_vector_z = B.prop_get('u_vector_z')

class SectionMeshList:
	def __init__(self, meshes:List['SectionMesh'] = [], area:float = 0.0, units:str = ''):
		self.meshes = meshes
		self.area = area
		self.units = units
	def store(self, B):
		B.objects_add('meshes', self.meshes)
		B.prop_add('area', self.area, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.meshes = B.objects_get('meshes', SectionMesh)
		self.area = B.prop_get('area')
		self.units = B.prop_get('units')

class SectionMeshNodes:
	def __init__(self, coord_x:List[float] = [], coord_y:List[float] = [], coord_z:List[float] = [], coord_u:List[float] = [], coord_v:List[float] = [], source_node_1:List[int] = [], source_node_2:List[int] = [], intersection_t:List[float] = [], u_vector_x:float = 0.0, u_vector_y:float = 0.0, u_vector_z:float = 0.0, v_vector_x:float = 0.0, v_vector_y:float = 0.0, v_vector_z:float = 0.0, uv_origin_x:float = 0.0, uv_origin_y:float = 0.0, uv_origin_z:float = 0.0, units:str = ''):
		self.coord_x = coord_x.copy()
		self.coord_y = coord_y.copy()
		self.coord_z = coord_z.copy()
		self.coord_u = coord_u.copy()
		self.coord_v = coord_v.copy()
		self.source_node_1 = source_node_1.copy()
		self.source_node_2 = source_node_2.copy()
		self.intersection_t = intersection_t.copy()
		self.u_vector_x = u_vector_x
		self.u_vector_y = u_vector_y
		self.u_vector_z = u_vector_z
		self.v_vector_x = v_vector_x
		self.v_vector_y = v_vector_y
		self.v_vector_z = v_vector_z
		self.uv_origin_x = uv_origin_x
		self.uv_origin_y = uv_origin_y
		self.uv_origin_z = uv_origin_z
		self.units = units
	def store(self, B):
		B.prop_add('coord_x', self.coord_x, BScmFmt.TypeDoubleList)
		B.prop_add('coord_y', self.coord_y, BScmFmt.TypeDoubleList)
		B.prop_add('coord_z', self.coord_z, BScmFmt.TypeDoubleList)
		B.prop_add('coord_u', self.coord_u, BScmFmt.TypeDoubleList)
		B.prop_add('coord_v', self.coord_v, BScmFmt.TypeDoubleList)
		B.prop_add('source_node_1', self.source_node_1, BScmFmt.TypeIntList)
		B.prop_add('source_node_2', self.source_node_2, BScmFmt.TypeIntList)
		B.prop_add('intersection_t', self.intersection_t, BScmFmt.TypeDoubleList)
		B.prop_add('u_vector_x', self.u_vector_x, BScmFmt.TypeDouble)
		B.prop_add('u_vector_y', self.u_vector_y, BScmFmt.TypeDouble)
		B.prop_add('u_vector_z', self.u_vector_z, BScmFmt.TypeDouble)
		B.prop_add('v_vector_x', self.v_vector_x, BScmFmt.TypeDouble)
		B.prop_add('v_vector_y', self.v_vector_y, BScmFmt.TypeDouble)
		B.prop_add('v_vector_z', self.v_vector_z, BScmFmt.TypeDouble)
		B.prop_add('uv_origin_x', self.uv_origin_x, BScmFmt.TypeDouble)
		B.prop_add('uv_origin_y', self.uv_origin_y, BScmFmt.TypeDouble)
		B.prop_add('uv_origin_z', self.uv_origin_z, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.coord_x = B.prop_get('coord_x')
		self.coord_y = B.prop_get('coord_y')
		self.coord_z = B.prop_get('coord_z')
		self.coord_u = B.prop_get('coord_u')
		self.coord_v = B.prop_get('coord_v')
		self.source_node_1 = B.prop_get('source_node_1')
		self.source_node_2 = B.prop_get('source_node_2')
		self.intersection_t = B.prop_get('intersection_t')
		self.u_vector_x = B.prop_get('u_vector_x')
		self.u_vector_y = B.prop_get('u_vector_y')
		self.u_vector_z = B.prop_get('u_vector_z')
		self.v_vector_x = B.prop_get('v_vector_x')
		self.v_vector_y = B.prop_get('v_vector_y')
		self.v_vector_z = B.prop_get('v_vector_z')
		self.uv_origin_x = B.prop_get('uv_origin_x')
		self.uv_origin_y = B.prop_get('uv_origin_y')
		self.uv_origin_z = B.prop_get('uv_origin_z')
		self.units = B.prop_get('units')

class SectionMeshTriangles:
	def __init__(self, node_1:List[int] = [], node_2:List[int] = [], node_3:List[int] = []):
		self.node_1 = node_1.copy()
		self.node_2 = node_2.copy()
		self.node_3 = node_3.copy()
	def store(self, B):
		B.prop_add('node_1', self.node_1, BScmFmt.TypeIntList)
		B.prop_add('node_2', self.node_2, BScmFmt.TypeIntList)
		B.prop_add('node_3', self.node_3, BScmFmt.TypeIntList)
	def load(self, B):
		self.node_1 = B.prop_get('node_1')
		self.node_2 = B.prop_get('node_2')
		self.node_3 = B.prop_get('node_3')

class SectionMeshBound:
	def __init__(self, nodes:List[int] = [], nodes_is_on_edge:List[bool] = [], length:float = 0.0, area:float = 0.0, units:str = ''):
		self.nodes = nodes.copy()
		self.nodes_is_on_edge = nodes_is_on_edge.copy()
		self.length = length
		self.area = area
		self.units = units
	def store(self, B):
		B.prop_add('nodes', self.nodes, BScmFmt.TypeIntList)
		B.prop_add('nodes_is_on_edge', self.nodes_is_on_edge, BScmFmt.TypeBoolList)
		B.prop_add('length', self.length, BScmFmt.TypeDouble)
		B.prop_add('area', self.area, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.nodes = B.prop_get('nodes')
		self.nodes_is_on_edge = B.prop_get('nodes_is_on_edge')
		self.length = B.prop_get('length')
		self.area = B.prop_get('area')
		self.units = B.prop_get('units')

class SectionMesh:
	def __init__(self, bounds:List['SectionMeshBound'] = [], area:float = 0.0, units:str = ''):
		self.nodes = SectionMeshNodes()
		self.triangles = SectionMeshTriangles()
		self.bounds = bounds
		self.area = area
		self.units = units
	def store(self, B):
		B.object_add('nodes', self.nodes)
		B.object_add('triangles', self.triangles)
		B.objects_add('bounds', self.bounds)
		B.prop_add('area', self.area, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.nodes = B.object_get('nodes', SectionMeshNodes)
		self.triangles = B.object_get('triangles', SectionMeshTriangles)
		self.bounds = B.objects_get('bounds', SectionMeshBound)
		self.area = B.prop_get('area')
		self.units = B.prop_get('units')

class SectionPlane3P:
	def __init__(self, x1:float = 0.0, y1:float = 0.0, z1:float = 0.0, x2:float = 0.0, y2:float = 0.0, z2:float = 0.0, x3:float = 0.0, y3:float = 0.0, z3:float = 0.0, id:int = -1, offset:float = 0, reverse_cut:bool = False):
		self.x1 = x1
		self.y1 = y1
		self.z1 = z1
		self.x2 = x2
		self.y2 = y2
		self.z2 = z2
		self.x3 = x3
		self.y3 = y3
		self.z3 = z3
		self.id = id
		self.offset = offset
		self.reverse_cut = reverse_cut
	def store(self, B):
		B.prop_add('x1', self.x1, BScmFmt.TypeDouble)
		B.prop_add('y1', self.y1, BScmFmt.TypeDouble)
		B.prop_add('z1', self.z1, BScmFmt.TypeDouble)
		B.prop_add('x2', self.x2, BScmFmt.TypeDouble)
		B.prop_add('y2', self.y2, BScmFmt.TypeDouble)
		B.prop_add('z2', self.z2, BScmFmt.TypeDouble)
		B.prop_add('x3', self.x3, BScmFmt.TypeDouble)
		B.prop_add('y3', self.y3, BScmFmt.TypeDouble)
		B.prop_add('z3', self.z3, BScmFmt.TypeDouble)
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('offset', self.offset, BScmFmt.TypeDouble)
		B.prop_add('reverse_cut', self.reverse_cut, BScmFmt.TypeBool)
	def load(self, B):
		self.x1 = B.prop_get('x1')
		self.y1 = B.prop_get('y1')
		self.z1 = B.prop_get('z1')
		self.x2 = B.prop_get('x2')
		self.y2 = B.prop_get('y2')
		self.z2 = B.prop_get('z2')
		self.x3 = B.prop_get('x3')
		self.y3 = B.prop_get('y3')
		self.z3 = B.prop_get('z3')
		self.id = B.prop_get('id')
		self.offset = B.prop_get('offset')
		self.reverse_cut = B.prop_get('reverse_cut')

class SectionPlanePN:
	def __init__(self, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0, normal_x:float = 0.0, normal_y:float = 0.0, normal_z:float = 0.0, id:int = -1, offset:float = 0, reverse_cut:bool = False):
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
		self.normal_x = normal_x
		self.normal_y = normal_y
		self.normal_z = normal_z
		self.id = id
		self.offset = offset
		self.reverse_cut = reverse_cut
	def store(self, B):
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
		B.prop_add('normal_x', self.normal_x, BScmFmt.TypeDouble)
		B.prop_add('normal_y', self.normal_y, BScmFmt.TypeDouble)
		B.prop_add('normal_z', self.normal_z, BScmFmt.TypeDouble)
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('offset', self.offset, BScmFmt.TypeDouble)
		B.prop_add('reverse_cut', self.reverse_cut, BScmFmt.TypeBool)
	def load(self, B):
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.normal_x = B.prop_get('normal_x')
		self.normal_y = B.prop_get('normal_y')
		self.normal_z = B.prop_get('normal_z')
		self.id = B.prop_get('id')
		self.offset = B.prop_get('offset')
		self.reverse_cut = B.prop_get('reverse_cut')

class SectionPlane:
	def __init__(self, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0, normal_x:float = 0.0, normal_y:float = 0.0, normal_z:float = 0.0, defined:bool = False, units:str = ''):
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
		self.normal_x = normal_x
		self.normal_y = normal_y
		self.normal_z = normal_z
		self.defined = defined
		self.units = units
	def store(self, B):
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
		B.prop_add('normal_x', self.normal_x, BScmFmt.TypeDouble)
		B.prop_add('normal_y', self.normal_y, BScmFmt.TypeDouble)
		B.prop_add('normal_z', self.normal_z, BScmFmt.TypeDouble)
		B.prop_add('defined', self.defined, BScmFmt.TypeBool)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.normal_x = B.prop_get('normal_x')
		self.normal_y = B.prop_get('normal_y')
		self.normal_z = B.prop_get('normal_z')
		self.defined = B.prop_get('defined')
		self.units = B.prop_get('units')

class Session:
	def __init__(self, session_id:int = 0, process_id:int = 0, is_api_connected:bool = False, is_started_by_api:bool = False, last_api_connected_app:str = '', last_api_connected_app_pid:int = 0, window_id:int = 0, last_window_activation_time:float = 0.0, last_window_deactivation_time:float = 0.0, is_visible:bool = False, is_simulation_running:bool = False, opened_project:str = '', qform_dir:str = ''):
		self.session_id = session_id
		self.process_id = process_id
		self.is_api_connected = is_api_connected
		self.is_started_by_api = is_started_by_api
		self.last_api_connected_app = last_api_connected_app
		self.last_api_connected_app_pid = last_api_connected_app_pid
		self.window_id = window_id
		self.last_window_activation_time = last_window_activation_time
		self.last_window_deactivation_time = last_window_deactivation_time
		self.is_visible = is_visible
		self.is_simulation_running = is_simulation_running
		self.opened_project = opened_project
		self.qform_dir = qform_dir
	def store(self, B):
		B.prop_add('session_id', self.session_id, BScmFmt.TypeInt)
		B.prop_add('process_id', self.process_id, BScmFmt.TypeInt)
		B.prop_add('is_api_connected', self.is_api_connected, BScmFmt.TypeBool)
		B.prop_add('is_started_by_api', self.is_started_by_api, BScmFmt.TypeBool)
		B.prop_add('last_api_connected_app', self.last_api_connected_app, BScmFmt.TypeStr)
		B.prop_add('last_api_connected_app_pid', self.last_api_connected_app_pid, BScmFmt.TypeInt)
		B.prop_add('window_id', self.window_id, BScmFmt.TypeInt)
		B.prop_add('last_window_activation_time', self.last_window_activation_time, BScmFmt.TypeDouble)
		B.prop_add('last_window_deactivation_time', self.last_window_deactivation_time, BScmFmt.TypeDouble)
		B.prop_add('is_visible', self.is_visible, BScmFmt.TypeBool)
		B.prop_add('is_simulation_running', self.is_simulation_running, BScmFmt.TypeBool)
		B.prop_add('opened_project', self.opened_project, BScmFmt.TypeStr)
		B.prop_add('qform_dir', self.qform_dir, BScmFmt.TypeStr)
	def load(self, B):
		self.session_id = B.prop_get('session_id')
		self.process_id = B.prop_get('process_id')
		self.is_api_connected = B.prop_get('is_api_connected')
		self.is_started_by_api = B.prop_get('is_started_by_api')
		self.last_api_connected_app = B.prop_get('last_api_connected_app')
		self.last_api_connected_app_pid = B.prop_get('last_api_connected_app_pid')
		self.window_id = B.prop_get('window_id')
		self.last_window_activation_time = B.prop_get('last_window_activation_time')
		self.last_window_deactivation_time = B.prop_get('last_window_deactivation_time')
		self.is_visible = B.prop_get('is_visible')
		self.is_simulation_running = B.prop_get('is_simulation_running')
		self.opened_project = B.prop_get('opened_project')
		self.qform_dir = B.prop_get('qform_dir')

class SessionList:
	def __init__(self, sessions:List['Session'] = []):
		self.sessions = sessions
	def store(self, B):
		B.objects_add('sessions', self.sessions)
	def load(self, B):
		self.sessions = B.objects_get('sessions', Session)

class SleepTime:
	def __init__(self, seconds:float = 0.0):
		self.seconds = seconds
	def store(self, B):
		B.prop_add('seconds', self.seconds, BScmFmt.TypeDouble)
	def load(self, B):
		self.seconds = B.prop_get('seconds')

class MainSimulationResult:
	def __init__(self, status:StatusCode = StatusCode.Ok, finished_calculation_unit:CalculationUnit = CalculationUnit.Nothing):
		self.status = status
		self.finished_calculation_unit = finished_calculation_unit
	def store(self, B):
		B.enum_add('status', self.status)
		B.enum_add('finished_calculation_unit', self.finished_calculation_unit)
	def load(self, B):
		self.status = B.enum_get('status', StatusCode)
		self.finished_calculation_unit = B.enum_get('finished_calculation_unit', CalculationUnit)

class OptionalGlobalItemId:
	def __init__(self, id:int = -1, operation:int = -1):
		self.id = id
		self.operation = operation
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('operation', self.operation, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.operation = B.prop_get('operation')

class SimulationState:
	def __init__(self, simulation_completed:bool = False, process_chain:int = 0, operation_index_in_chain:int = 0, operation:int = 0, blow:int = 0, blow_record_count:int = 0, blow_progress:float = 0.0, units:str = ''):
		self.simulation_completed = simulation_completed
		self.process_chain = process_chain
		self.operation_index_in_chain = operation_index_in_chain
		self.operation = operation
		self.blow = blow
		self.blow_record_count = blow_record_count
		self.blow_progress = blow_progress
		self.units = units
	def store(self, B):
		B.prop_add('simulation_completed', self.simulation_completed, BScmFmt.TypeBool)
		B.prop_add('process_chain', self.process_chain, BScmFmt.TypeInt)
		B.prop_add('operation_index_in_chain', self.operation_index_in_chain, BScmFmt.TypeInt)
		B.prop_add('operation', self.operation, BScmFmt.TypeInt)
		B.prop_add('blow', self.blow, BScmFmt.TypeInt)
		B.prop_add('blow_record_count', self.blow_record_count, BScmFmt.TypeInt)
		B.prop_add('blow_progress', self.blow_progress, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.simulation_completed = B.prop_get('simulation_completed')
		self.process_chain = B.prop_get('process_chain')
		self.operation_index_in_chain = B.prop_get('operation_index_in_chain')
		self.operation = B.prop_get('operation')
		self.blow = B.prop_get('blow')
		self.blow_record_count = B.prop_get('blow_record_count')
		self.blow_progress = B.prop_get('blow_progress')
		self.units = B.prop_get('units')

class SystemStateId:
	def __init__(self, record:float = -1):
		self.record = record
	def store(self, B):
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
	def load(self, B):
		self.record = B.prop_get('record')

class ExtrusionState:
	def __init__(self, force:float = 0.0, fill_time:float = 0.0, euler_min_z:float = 0.0, units:str = ''):
		self.force = force
		self.fill_time = fill_time
		self.euler_min_z = euler_min_z
		self.units = units
	def store(self, B):
		B.prop_add('force', self.force, BScmFmt.TypeDouble)
		B.prop_add('fill_time', self.fill_time, BScmFmt.TypeDouble)
		B.prop_add('euler_min_z', self.euler_min_z, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.force = B.prop_get('force')
		self.fill_time = B.prop_get('fill_time')
		self.euler_min_z = B.prop_get('euler_min_z')
		self.units = B.prop_get('units')

class MeshStateId:
	def __init__(self, record:float = -1, id:int = 0, type:ObjectType = ObjectType.Nothing, mesh_index:int = 0):
		self.record = record
		self.id = id
		self.type = type
		self.mesh_index = mesh_index
	def store(self, B):
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('type', self.type)
		B.prop_add('mesh_index', self.mesh_index, BScmFmt.TypeInt)
	def load(self, B):
		self.record = B.prop_get('record')
		self.id = B.prop_get('id')
		self.type = B.enum_get('type', ObjectType)
		self.mesh_index = B.prop_get('mesh_index')

class MeshState:
	def __init__(self, volumetric_elements:int = 0, surface_elements:int = 0, internal_nodes:int = 0, surface_nodes:int = 0, total_nodes:int = 0, dim:int = 0, record:float = 0.0):
		self.volumetric_elements = volumetric_elements
		self.surface_elements = surface_elements
		self.internal_nodes = internal_nodes
		self.surface_nodes = surface_nodes
		self.total_nodes = total_nodes
		self.dim = dim
		self.record = record
	def store(self, B):
		B.prop_add('volumetric_elements', self.volumetric_elements, BScmFmt.TypeInt)
		B.prop_add('surface_elements', self.surface_elements, BScmFmt.TypeInt)
		B.prop_add('internal_nodes', self.internal_nodes, BScmFmt.TypeInt)
		B.prop_add('surface_nodes', self.surface_nodes, BScmFmt.TypeInt)
		B.prop_add('total_nodes', self.total_nodes, BScmFmt.TypeInt)
		B.prop_add('dim', self.dim, BScmFmt.TypeInt)
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
	def load(self, B):
		self.volumetric_elements = B.prop_get('volumetric_elements')
		self.surface_elements = B.prop_get('surface_elements')
		self.internal_nodes = B.prop_get('internal_nodes')
		self.surface_nodes = B.prop_get('surface_nodes')
		self.total_nodes = B.prop_get('total_nodes')
		self.dim = B.prop_get('dim')
		self.record = B.prop_get('record')

class SystemState:
	def __init__(self, record:float = 0.0, time:float = 0.0, process_time:float = 0.0, time_step:float = 0.0, units:str = '', iteration_count:int = 0, calculation_time:float = 0.0, total_calculation_time:float = 0.0, progress:float = 0.0):
		self.record = record
		self.time = time
		self.process_time = process_time
		self.time_step = time_step
		self.units = units
		self.iteration_count = iteration_count
		self.calculation_time = calculation_time
		self.total_calculation_time = total_calculation_time
		self.progress = progress
	def store(self, B):
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
		B.prop_add('time', self.time, BScmFmt.TypeDouble)
		B.prop_add('process_time', self.process_time, BScmFmt.TypeDouble)
		B.prop_add('time_step', self.time_step, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
		B.prop_add('iteration_count', self.iteration_count, BScmFmt.TypeInt)
		B.prop_add('calculation_time', self.calculation_time, BScmFmt.TypeDouble)
		B.prop_add('total_calculation_time', self.total_calculation_time, BScmFmt.TypeDouble)
		B.prop_add('progress', self.progress, BScmFmt.TypeDouble)
	def load(self, B):
		self.record = B.prop_get('record')
		self.time = B.prop_get('time')
		self.process_time = B.prop_get('process_time')
		self.time_step = B.prop_get('time_step')
		self.units = B.prop_get('units')
		self.iteration_count = B.prop_get('iteration_count')
		self.calculation_time = B.prop_get('calculation_time')
		self.total_calculation_time = B.prop_get('total_calculation_time')
		self.progress = B.prop_get('progress')

class ToolStateId:
	def __init__(self, record:float = -1, id:int = 0, type:ObjectType = ObjectType.Nothing):
		self.record = record
		self.id = id
		self.type = type
	def store(self, B):
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('type', self.type)
	def load(self, B):
		self.record = B.prop_get('record')
		self.id = B.prop_get('id')
		self.type = B.enum_get('type', ObjectType)

class ToolState:
	def __init__(self, record:float = 0.0, load:float = 0.0, energy:float = 0.0, displacement:float = 0.0, power:float = 0.0, torque_1:float = 0.0, torque_2:float = 0.0, work:float = 0.0, velocity:float = 0.0, units:str = ''):
		self.record = record
		self.load = load
		self.energy = energy
		self.displacement = displacement
		self.power = power
		self.torque_1 = torque_1
		self.torque_2 = torque_2
		self.work = work
		self.velocity = velocity
		self.units = units
	def store(self, B):
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
		B.prop_add('load', self.load, BScmFmt.TypeDouble)
		B.prop_add('energy', self.energy, BScmFmt.TypeDouble)
		B.prop_add('displacement', self.displacement, BScmFmt.TypeDouble)
		B.prop_add('power', self.power, BScmFmt.TypeDouble)
		B.prop_add('torque_1', self.torque_1, BScmFmt.TypeDouble)
		B.prop_add('torque_2', self.torque_2, BScmFmt.TypeDouble)
		B.prop_add('work', self.work, BScmFmt.TypeDouble)
		B.prop_add('velocity', self.velocity, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.record = B.prop_get('record')
		self.load = B.prop_get('load')
		self.energy = B.prop_get('energy')
		self.displacement = B.prop_get('displacement')
		self.power = B.prop_get('power')
		self.torque_1 = B.prop_get('torque_1')
		self.torque_2 = B.prop_get('torque_2')
		self.work = B.prop_get('work')
		self.velocity = B.prop_get('velocity')
		self.units = B.prop_get('units')

class WorkpieceStateId:
	def __init__(self, record:float = -1, object_id:int = 0):
		self.record = record
		self.object_id = object_id
	def store(self, B):
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
	def load(self, B):
		self.record = B.prop_get('record')
		self.object_id = B.prop_get('object_id')

class WorkpieceState:
	def __init__(self, record:float = 0.0, volume:float = 0.0, units:str = '', with_laps:bool = False):
		self.record = record
		self.volume = volume
		self.units = units
		self.with_laps = with_laps
	def store(self, B):
		B.prop_add('record', self.record, BScmFmt.TypeDouble)
		B.prop_add('volume', self.volume, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
		B.prop_add('with_laps', self.with_laps, BScmFmt.TypeBool)
	def load(self, B):
		self.record = B.prop_get('record')
		self.volume = B.prop_get('volume')
		self.units = B.prop_get('units')
		self.with_laps = B.prop_get('with_laps')

class StopCondParams:
	def __init__(self, id:int = -1, type:StopCondType = StopCondType.Distance):
		self.id = id
		self.type = type
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('type', self.type)
	def load(self, B):
		self.id = B.prop_get('id')
		self.type = B.enum_get('type', StopCondType)

class StopCondDistance:
	def __init__(self, id:int = -1, distance:float = 0.0, object_1_type:ObjectType = ObjectType.Nothing, object_1_id:int = 0, object_2_type:ObjectType = ObjectType.Nothing, object_2_id:int = 0):
		self.id = id
		self.distance = distance
		self.object_1_type = object_1_type
		self.object_1_id = object_1_id
		self.object_2_type = object_2_type
		self.object_2_id = object_2_id
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('distance', self.distance, BScmFmt.TypeDouble)
		B.enum_add('object_1_type', self.object_1_type)
		B.prop_add('object_1_id', self.object_1_id, BScmFmt.TypeInt)
		B.enum_add('object_2_type', self.object_2_type)
		B.prop_add('object_2_id', self.object_2_id, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.distance = B.prop_get('distance')
		self.object_1_type = B.enum_get('object_1_type', ObjectType)
		self.object_1_id = B.prop_get('object_1_id')
		self.object_2_type = B.enum_get('object_2_type', ObjectType)
		self.object_2_id = B.prop_get('object_2_id')

class StopCondFinPos:
	def __init__(self, id:int = -1, tool_type:ObjectType = ObjectType.Nothing, tool_number:int = 0):
		self.id = id
		self.tool_type = tool_type
		self.tool_number = tool_number
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('tool_type', self.tool_type)
		B.prop_add('tool_number', self.tool_number, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.tool_type = B.enum_get('tool_type', ObjectType)
		self.tool_number = B.prop_get('tool_number')

class StopCondMaxLoad:
	def __init__(self, id:int = -1, tool_type:ObjectType = ObjectType.Nothing, tool_number:int = 0, max_load:float = 0.0):
		self.id = id
		self.tool_type = tool_type
		self.tool_number = tool_number
		self.max_load = max_load
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('tool_type', self.tool_type)
		B.prop_add('tool_number', self.tool_number, BScmFmt.TypeInt)
		B.prop_add('max_load', self.max_load, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.tool_type = B.enum_get('tool_type', ObjectType)
		self.tool_number = B.prop_get('tool_number')
		self.max_load = B.prop_get('max_load')

class StopCondRotation:
	def __init__(self, id:int = -1, tool_type:ObjectType = ObjectType.Nothing, tool_number:int = 0, angle:float = 0.0, axis:int = 0):
		self.id = id
		self.tool_type = tool_type
		self.tool_number = tool_number
		self.angle = angle
		self.axis = axis
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('tool_type', self.tool_type)
		B.prop_add('tool_number', self.tool_number, BScmFmt.TypeInt)
		B.prop_add('angle', self.angle, BScmFmt.TypeDouble)
		B.prop_add('axis', self.axis, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.tool_type = B.enum_get('tool_type', ObjectType)
		self.tool_number = B.prop_get('tool_number')
		self.angle = B.prop_get('angle')
		self.axis = B.prop_get('axis')

class StopCondStroke:
	def __init__(self, id:int = -1, tool_type:ObjectType = ObjectType.Nothing, tool_number:int = 0, stroke:float = 0.0):
		self.id = id
		self.tool_type = tool_type
		self.tool_number = tool_number
		self.stroke = stroke
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('tool_type', self.tool_type)
		B.prop_add('tool_number', self.tool_number, BScmFmt.TypeInt)
		B.prop_add('stroke', self.stroke, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.tool_type = B.enum_get('tool_type', ObjectType)
		self.tool_number = B.prop_get('tool_number')
		self.stroke = B.prop_get('stroke')

class StopCondTime:
	def __init__(self, id:int = -1, time:float = 0.0):
		self.id = id
		self.time = time
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('time', self.time, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.time = B.prop_get('time')

class StopCond:
	def __init__(self, type:StopCondType = StopCondType.Distance):
		self.type = type
	def store(self, B):
		B.enum_add('type', self.type)
	def load(self, B):
		self.type = B.enum_get('type', StopCondType)

class StopCondFieldValue:
	def __init__(self, id:int = -1, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, bound_value:float = 0.0, bound_type:SolidBodyNodeValueCondition_ValueType = SolidBodyNodeValueCondition_ValueType.MIN_VALUE, region_in_object:SolidBodyNodeValueCondition_ValueRegionInBody = SolidBodyNodeValueCondition_ValueRegionInBody.ANY_NODE):
		self.id = id
		self.object_type = object_type
		self.object_id = object_id
		self.bound_value = bound_value
		self.bound_type = bound_type
		self.region_in_object = region_in_object
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('bound_value', self.bound_value, BScmFmt.TypeDouble)
		B.enum_add('bound_type', self.bound_type)
		B.enum_add('region_in_object', self.region_in_object)
	def load(self, B):
		self.id = B.prop_get('id')
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.bound_value = B.prop_get('bound_value')
		self.bound_type = B.enum_get('bound_type', SolidBodyNodeValueCondition_ValueType)
		self.region_in_object = B.enum_get('region_in_object', SolidBodyNodeValueCondition_ValueRegionInBody)

class SubroutineCreate:
	def __init__(self, id:int = -1, type:SubRoutineType = SubRoutineType.lua, file:str = '', solve_in_process:bool = False):
		self.id = id
		self.type = type
		self.file = file
		self.solve_in_process = solve_in_process
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('type', self.type)
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.prop_add('solve_in_process', self.solve_in_process, BScmFmt.TypeBool)
	def load(self, B):
		self.id = B.prop_get('id')
		self.type = B.enum_get('type', SubRoutineType)
		self.file = B.prop_get('file')
		self.solve_in_process = B.prop_get('solve_in_process')

class Subroutine:
	def __init__(self, used_subroutines:List[int] = [], parameters:List[str] = [], input_fields:List[str] = [], output_fields:List[str] = [], type:SubRoutineType = SubRoutineType.lua, file:str = '', solve_in_process:bool = False):
		self.used_subroutines = used_subroutines.copy()
		self.parameters = parameters.copy()
		self.input_fields = input_fields.copy()
		self.output_fields = output_fields.copy()
		self.type = type
		self.file = file
		self.solve_in_process = solve_in_process
	def store(self, B):
		B.prop_add('used_subroutines', self.used_subroutines, BScmFmt.TypeIntList)
		B.strings_add('parameters', self.parameters)
		B.strings_add('input_fields', self.input_fields)
		B.strings_add('output_fields', self.output_fields)
		B.enum_add('type', self.type)
		B.prop_add('file', self.file, BScmFmt.TypeStr)
		B.prop_add('solve_in_process', self.solve_in_process, BScmFmt.TypeBool)
	def load(self, B):
		self.used_subroutines = B.prop_get('used_subroutines')
		self.parameters = B.strings_get('parameters')
		self.input_fields = B.strings_get('input_fields')
		self.output_fields = B.strings_get('output_fields')
		self.type = B.enum_get('type', SubRoutineType)
		self.file = B.prop_get('file')
		self.solve_in_process = B.prop_get('solve_in_process')

class SubroutineParameter:
	def __init__(self, id:int = 0, parameter:str = '', value:float = 0.0):
		self.id = id
		self.parameter = parameter
		self.value = value
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('parameter', self.parameter, BScmFmt.TypeStr)
		B.prop_add('value', self.value, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.parameter = B.prop_get('parameter')
		self.value = B.prop_get('value')

class SymPlaneParams:
	def __init__(self, id:int = -1, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0, normal_x:float = 0.0, normal_y:float = 0.0, normal_z:float = 0.0):
		self.id = id
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
		self.normal_x = normal_x
		self.normal_y = normal_y
		self.normal_z = normal_z
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
		B.prop_add('normal_x', self.normal_x, BScmFmt.TypeDouble)
		B.prop_add('normal_y', self.normal_y, BScmFmt.TypeDouble)
		B.prop_add('normal_z', self.normal_z, BScmFmt.TypeDouble)
	def load(self, B):
		self.id = B.prop_get('id')
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.normal_x = B.prop_get('normal_x')
		self.normal_y = B.prop_get('normal_y')
		self.normal_z = B.prop_get('normal_z')

class SymPlaneByPoint:
	def __init__(self, id:int = -1, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0):
		self.id = id
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
		self.object_type = object_type
		self.object_id = object_id
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')

class SymPlaneByColor:
	def __init__(self, id:int = -1, color_R:int = 0, color_G:int = 0, color_B:int = 0, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0):
		self.id = id
		self.color_R = color_R
		self.color_G = color_G
		self.color_B = color_B
		self.object_type = object_type
		self.object_id = object_id
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('color_R', self.color_R, BScmFmt.TypeInt)
		B.prop_add('color_G', self.color_G, BScmFmt.TypeInt)
		B.prop_add('color_B', self.color_B, BScmFmt.TypeInt)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.color_R = B.prop_get('color_R')
		self.color_G = B.prop_get('color_G')
		self.color_B = B.prop_get('color_B')
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')

class UnitVector:
	def __init__(self, vector_x:float = 0.0, vector_y:float = 0.0, vector_z:float = 0.0, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0, units:str = ''):
		self.vector_x = vector_x
		self.vector_y = vector_y
		self.vector_z = vector_z
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
		self.units = units
	def store(self, B):
		B.prop_add('vector_x', self.vector_x, BScmFmt.TypeDouble)
		B.prop_add('vector_y', self.vector_y, BScmFmt.TypeDouble)
		B.prop_add('vector_z', self.vector_z, BScmFmt.TypeDouble)
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
		B.prop_add('units', self.units, BScmFmt.TypeStr)
	def load(self, B):
		self.vector_x = B.prop_get('vector_x')
		self.vector_y = B.prop_get('vector_y')
		self.vector_z = B.prop_get('vector_z')
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.units = B.prop_get('units')

class UnitSystem:
	def __init__(self, system:SystemOfUnits = SystemOfUnits.SI):
		self.system = system
	def store(self, B):
		B.enum_add('system', self.system)
	def load(self, B):
		self.system = B.enum_get('system', SystemOfUnits)

class NewTrackingContour:
	def __init__(self, index:int = -1, tracking_contours_id:int = 0, point_x:List[float] = [], point_y:List[float] = [], point_z:List[float] = []):
		self.index = index
		self.tracking_contours_id = tracking_contours_id
		self.point_x = point_x.copy()
		self.point_y = point_y.copy()
		self.point_z = point_z.copy()
	def store(self, B):
		B.prop_add('index', self.index, BScmFmt.TypeInt)
		B.prop_add('tracking_contours_id', self.tracking_contours_id, BScmFmt.TypeInt)
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDoubleList)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDoubleList)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDoubleList)
	def load(self, B):
		self.index = B.prop_get('index')
		self.tracking_contours_id = B.prop_get('tracking_contours_id')
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')

class OptionalObjectItemId:
	def __init__(self, id:int = -1, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0):
		self.id = id
		self.object_type = object_type
		self.object_id = object_id
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')

class TrackingGroup:
	def __init__(self, id:int = 0, name:str = ''):
		self.id = id
		self.name = name
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('name', self.name, BScmFmt.TypeStr)
	def load(self, B):
		self.id = B.prop_get('id')
		self.name = B.prop_get('name')

class TrackingLineParams:
	def __init__(self, id:int = -1, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, point1_x:float = 0.0, point1_y:float = 0.0, point1_z:float = 0.0, point2_x:float = 0.0, point2_y:float = 0.0, point2_z:float = 0.0, on_surface:bool = False):
		self.id = id
		self.object_type = object_type
		self.object_id = object_id
		self.point1_x = point1_x
		self.point1_y = point1_y
		self.point1_z = point1_z
		self.point2_x = point2_x
		self.point2_y = point2_y
		self.point2_z = point2_z
		self.on_surface = on_surface
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('point1_x', self.point1_x, BScmFmt.TypeDouble)
		B.prop_add('point1_y', self.point1_y, BScmFmt.TypeDouble)
		B.prop_add('point1_z', self.point1_z, BScmFmt.TypeDouble)
		B.prop_add('point2_x', self.point2_x, BScmFmt.TypeDouble)
		B.prop_add('point2_y', self.point2_y, BScmFmt.TypeDouble)
		B.prop_add('point2_z', self.point2_z, BScmFmt.TypeDouble)
		B.prop_add('on_surface', self.on_surface, BScmFmt.TypeBool)
	def load(self, B):
		self.id = B.prop_get('id')
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.point1_x = B.prop_get('point1_x')
		self.point1_y = B.prop_get('point1_y')
		self.point1_z = B.prop_get('point1_z')
		self.point2_x = B.prop_get('point2_x')
		self.point2_y = B.prop_get('point2_y')
		self.point2_z = B.prop_get('point2_z')
		self.on_surface = B.prop_get('on_surface')

class GlobalItemId:
	def __init__(self, id:int = 0, operation:int = -1):
		self.id = id
		self.operation = operation
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.prop_add('operation', self.operation, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.operation = B.prop_get('operation')

class TrackingLine:
	def __init__(self, points:List['MeshPoint'] = []):
		self.points = points
	def store(self, B):
		B.objects_add('points', self.points)
	def load(self, B):
		self.points = B.objects_get('points', MeshPoint)

class GlobalItemList:
	def __init__(self, objects:List['GlobalItemId'] = []):
		self.objects = objects
	def store(self, B):
		B.objects_add('objects', self.objects)
	def load(self, B):
		self.objects = B.objects_get('objects', GlobalItemId)

class TrackingPointParams:
	def __init__(self, id:int = -1, object_type:ObjectType = ObjectType.Nothing, object_id:int = 0, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0, on_surface:bool = False, group:int = 0):
		self.id = id
		self.object_type = object_type
		self.object_id = object_id
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
		self.on_surface = on_surface
		self.group = group
	def store(self, B):
		B.prop_add('id', self.id, BScmFmt.TypeInt)
		B.enum_add('object_type', self.object_type)
		B.prop_add('object_id', self.object_id, BScmFmt.TypeInt)
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
		B.prop_add('on_surface', self.on_surface, BScmFmt.TypeBool)
		B.prop_add('group', self.group, BScmFmt.TypeInt)
	def load(self, B):
		self.id = B.prop_get('id')
		self.object_type = B.enum_get('object_type', ObjectType)
		self.object_id = B.prop_get('object_id')
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.on_surface = B.prop_get('on_surface')
		self.group = B.prop_get('group')

class View:
	def __init__(self, point_x:float = 0.0, point_y:float = 0.0, point_z:float = 0.0, vector_x:float = 0.0, vector_y:float = 0.0, vector_z:float = 0.0):
		self.point_x = point_x
		self.point_y = point_y
		self.point_z = point_z
		self.vector_x = vector_x
		self.vector_y = vector_y
		self.vector_z = vector_z
	def store(self, B):
		B.prop_add('point_x', self.point_x, BScmFmt.TypeDouble)
		B.prop_add('point_y', self.point_y, BScmFmt.TypeDouble)
		B.prop_add('point_z', self.point_z, BScmFmt.TypeDouble)
		B.prop_add('vector_x', self.vector_x, BScmFmt.TypeDouble)
		B.prop_add('vector_y', self.vector_y, BScmFmt.TypeDouble)
		B.prop_add('vector_z', self.vector_z, BScmFmt.TypeDouble)
	def load(self, B):
		self.point_x = B.prop_get('point_x')
		self.point_y = B.prop_get('point_y')
		self.point_z = B.prop_get('point_z')
		self.vector_x = B.prop_get('vector_x')
		self.vector_y = B.prop_get('vector_y')
		self.vector_z = B.prop_get('vector_z')

class ViewOption:
	def __init__(self, option:ViewOptions = ViewOptions.ShowMinMax, value:bool = False):
		self.option = option
		self.value = value
	def store(self, B):
		B.enum_add('option', self.option)
		B.prop_add('value', self.value, BScmFmt.TypeBool)
	def load(self, B):
		self.option = B.enum_get('option', ViewOptions)
		self.value = B.prop_get('value')

class QFormStatus:
	def __init__(self, v):
		self.value = v
class QFormStatus:
	NotConnected		= QFormStatus('NotConnected')		# 0
	StartedByApi		= QFormStatus('StartedByApi')		# 1
	Attached			= QFormStatus('Attached')			# 2
	Detached			= QFormStatus('Detached')			# 3
	ClosedByUser		= QFormStatus('ClosedByUser')		# 4
	ClosedByApi			= QFormStatus('ClosedByApi')		# 5
	ClosedByException	= QFormStatus('ClosedByException')	# 6
	ConnectionLost		= QFormStatus('ConnectionLost')		# 7
	ConnectionError		= QFormStatus('ConnectionError')	# 8
	def from_code(code):
		if code == 0: return QFormStatus.NotConnected
		if code == 1: return QFormStatus.StartedByApi
		if code == 2: return QFormStatus.Attached
		if code == 3: return QFormStatus.Detached
		if code == 4: return QFormStatus.ClosedByUser
		if code == 5: return QFormStatus.ClosedByApi
		if code == 6: return QFormStatus.ClosedByException
		if code == 7: return QFormStatus.ConnectionLost
		if code == 8: return QFormStatus.ConnectionError

class _QFormStatus:
	def __init__(self):
		self.status = QFormStatus.NotConnected
	def store(self, B):
		B.enum_add('status', self.status)
	def load(self, B):
		self.status = B.enum_get('status', QFormStatus)

class _QFormAttach:
	def __init__(self, qform_pid, out_pipe, out_sem, in_pipe, in_sem):
		self.qform_pid = qform_pid
		self.out_pipe = out_pipe
		self.out_sem = out_sem
		self.in_pipe = in_pipe
		self.in_sem = in_sem
	def store(self, B):
		B.prop_add('qform_pid',	self.qform_pid, BScmFmt.TypeStr)
		B.prop_add('out_pipe',	self.out_pipe, BScmFmt.TypeStr)
		B.prop_add('out_sem',	self.out_sem, BScmFmt.TypeStr)
		B.prop_add('in_pipe',	self.in_pipe, BScmFmt.TypeStr)
		B.prop_add('in_sem',	self.in_sem, BScmFmt.TypeStr)
	def load(self, B):
		self.qform_pid	= B.prop_get('qform_pid')
		self.out_pipe	= B.prop_get('out_pipe')
		self.out_sem	= B.prop_get('out_sem')
		self.in_pipe	= B.prop_get('in_pipe')
		self.in_sem		= B.prop_get('in_sem')


class QForm:
	_ERR_STARTED		= 'QForm is already started'
	_MSG_START_HIDDEN	= -9
	_MSG_START			= -8
	_MSG_ATTACH			= -10
	_MSG_EXIT			= -1
	_MSG_DETACH			= -2
	_MSG_STATUS			= -3
	_MSG_HIDE_CONSOLE	= -12

	def __init__(self, instance_name = None):
		self._proc = None
		self._qform_dir = None
		self._instance_name = instance_name
		self._last_err = None
		self._last_err_func = None
		self._exceptions = True

	def exceptions_enable(self, Enable = True) -> None:
		self._exceptions = Enable

	def exceptions_disable(self, Disable = True) -> None:
		self._exceptions = not Disable

	def exceptions_enabled(self) -> bool:
		return self._exceptions

	def qform_dir(self) -> str:
		if (self.is_started_by_qform()): return os.getenv('CTL_MASTER_DIR')
		return self._qform_dir

	def qform_dir_set(self, dir) -> None:
		self._qform_dir = dir

	def last_error(self) -> str:
		return self._last_err

	def last_error_at_function(self) -> str:
		return self._last_err_func

	def last_error_clear(self) -> None:
		self._last_err = None
		self._last_err_func = None

	def instance_name(self) -> str:
		return self._instance_name

	def is_started_by_qform(self) -> bool:
		pid = os.getenv('CTL_MASTER_PID')
		return pid is not None

	def _error(self, ret, msg_or_ex, func = None):
		if type(msg_or_ex) is Exception:
			self._last_err = str(msg_or_ex)
		else:
			self._last_err = msg_or_ex
		self._last_err_func = func
		if self._exceptions:
			if type(msg_or_ex) is Exception:
				raise Exception(*msg_or_ex.args)
			else:
				raise Exception(msg_or_ex)
		return ret

	def _no_console():
		try:
			with open('CONIN$'):
				return False
		except:
			return True

	def _mk_args(self, attach, no_window, svc_mode):
		dir = self.qform_dir()
		if dir is None: raise Exception('QForm directory is not set')
		exe = dir + r'\QFormCon.exe'
		args = [exe, '-bscm', '-parent', str(os.getpid())]
		if self._instance_name is not None:
			args.append('-inst')
			args.append(urllib.parse.quote(self._instance_name))
		if attach: args.append('-attach')
		if no_window: args.append('-nowindow')
		if svc_mode: args.append('-svc')
		if QForm._no_console(): args.append('-noconsole')
		return args

	def _is_proc_started(self):
		if self._proc is None:
			return False
		poll = self._proc.poll()
		return poll is None

	def _proc_touch(self, attach, no_window, svc_mode):
		if self._is_proc_started(): return
		args = self._mk_args(attach, no_window, svc_mode)
		self._proc = subprocess.Popen(args, stdout = subprocess.PIPE, stdin = subprocess.PIPE)

	def qform_is_running(self) -> bool:
		st = self.qform_status()
		if st is QFormStatus.StartedByApi: return True
		if st is QFormStatus.Attached: return True
		return False

	def qform_status(self) -> QFormStatus:
		if self._proc is None:
			return QFormStatus.NotConnected
		poll = self._proc.poll()
		if poll is not None:
			return QFormStatus.from_code(poll)
		try:
			st = self._invoke(QForm._MSG_STATUS, 'qform_status', None, _QFormStatus)
			return st.status
		except Exception as ex:
			return QFormStatus.NotConnected

	def qform_is_closed_by_user(self) -> bool:
		st = self.qform_status()
		return st is QFormStatus.ClosedByUser

	def qform_is_detached(self) -> bool:
		st = self.qform_status()
		return st is QFormStatus.Detached

	def qform_is_attached(self) -> bool:
		st = self.qform_status()
		return st is QFormStatus.Attached

	def qform_is_started_by_api(self) -> bool:
		st = self.qform_status()
		return st is QFormStatus.StartedByApi

	def _qform_start(self, msg, arg, func):
		if self.qform_is_running():
			return self._error(False, QForm._ERR_STARTED, func)
		try:
			self._proc_touch(False, False, True)
			ok = self._invoke(msg, func, arg, None)
			return ok
		except Exception as ex:
			return self._error(False, ex, func)

	def qform_start(self, no_window = False) -> bool:
		msg = QForm._MSG_START
		if no_window: msg = QForm._MSG_START_HIDDEN
		return self._qform_start(msg, None, 'qform_start')

	def qform_attach(self) -> bool:
		pid = os.getenv('CTL_MASTER_PID')
		outp = os.getenv('CTL_SLAVE_OUT')
		outs = os.getenv('CTL_SLAVE_OUT_SEM') 
		inp = os.getenv('CTL_SLAVE_IN')
		ins = os.getenv('CTL_SLAVE_IN_SEM') 
		arg = _QFormAttach(pid, outp, outs, inp, ins)
		return self._qform_start(QForm._MSG_ATTACH, arg, 'qform_attach')

	def qform_start_or_attach(self, no_window = False) -> bool:
		if self.is_started_by_qform():
			return self.qform_attach()
		return self.qform_start(no_window)

	def _invoke(self, svc, cmd, arg, Ret):
		if svc < 0:
			self._proc_touch(False, False, True)

		if arg is None and svc < 0:
			BScmFmt.write_int(self._proc.stdin, svc)
		else:
			msg = BScm()
			if arg is not None:
				arg.store(msg)

			msz = 0
			if svc < 0:
				msg.value_set_int(svc)
				msz = msg.byte_size()
				msz = -(1000 + msz)
			else:
				msg.value_set_str(cmd)
				msz = msg.byte_size()

			BScmFmt.write_int(self._proc.stdin, msz)
			msg.write(self._proc.stdin)

		self._proc.stdin.flush()
		response = BScm()
		response.read(self._proc.stdout)
		if response.value != 'ok':
			raise Exception(response.value)
		if Ret is None:
			return True
		ret = Ret()
		ret.load(response)
		return ret

	def invoke(self, svc, cmd, arg, Ret) -> bool:
		try:
			return self._invoke(svc, cmd, arg, Ret)
		except Exception as ex:
			rv = None
			if Ret is None: rv = False
			return self._error(rv, ex, cmd)

	def _disconnect(self, msg):
		if self._proc is None:
			return
		BScmFmt.write_int(self._proc.stdin, msg)
		self._proc.stdin.flush()
		self._proc = None
		self._attached = False

	def qform_close(self) -> None:
		self._disconnect(QForm._MSG_EXIT)

	def qform_detach(self) -> None:
		self._disconnect(QForm._MSG_DETACH)

	def qform_close_or_detach(self) -> None:
		st = self.qform_status()
		if st is QFormStatus.StartedByApi:
			self.qform_close();
		else:
			self.qform_detach()

	def active_field_get(self) -> FieldId:
		return self.invoke(0, 'active_field_get', None, FieldId)
	def active_field_reset(self) -> bool:
		return self.invoke(0, 'active_field_reset', None, None)
	def active_field_set(self, arg:FieldId) -> bool:
		return self.invoke(0, 'active_field_set', arg, None)
	def assembled_tool_create(self, arg:AssembledTool) -> ItemId:
		return self.invoke(0, 'assembled_tool_create', arg, ItemId)
	def assembled_tool_get(self, arg:ItemId) -> AssembledTool:
		return self.invoke(0, 'assembled_tool_get', arg, AssembledTool)
	def assembled_tool_split(self, arg:ItemId) -> bool:
		return self.invoke(0, 'assembled_tool_split', arg, None)
	def assembled_tools_get(self) -> ItemList:
		return self.invoke(0, 'assembled_tools_get', None, ItemList)
	def async_calculate_tools(self, arg:ObjectList) -> SimulationResult:
		return self.invoke(0, 'async_calculate_tools', arg, SimulationResult)
	def async_calculate_tools_coupled(self) -> SimulationResult:
		return self.invoke(0, 'async_calculate_tools_coupled', None, SimulationResult)
	def async_execute_subroutines(self, arg:SubroutineCalculationMode) -> bool:
		return self.invoke(0, 'async_execute_subroutines', arg, None)
	def async_execute_tracking(self, arg:TrackingCalculationMode) -> bool:
		return self.invoke(0, 'async_execute_tracking', arg, None)
	def async_start_simulation(self, arg:SimulationParams) -> bool:
		return self.invoke(0, 'async_start_simulation', arg, None)
	def async_state(self) -> AsyncState:
		return self.invoke(0, 'async_state', None, AsyncState)
	def async_stop_simulation(self) -> bool:
		return self.invoke(0, 'async_stop_simulation', None, None)
	def async_wait(self, arg:AsyncWaitingParams) -> AsyncEvent:
		return self.invoke(0, 'async_wait', arg, AsyncEvent)
	def available_fields_get(self, arg:FieldGroupId) -> FieldIdList:
		return self.invoke(0, 'available_fields_get', arg, FieldIdList)
	def axis_calculate(self, arg:ObjectId) -> ObjectAxis:
		return self.invoke(0, 'axis_calculate', arg, ObjectAxis)
	def axis_delete(self, arg:ObjectAxisId) -> bool:
		return self.invoke(0, 'axis_delete', arg, None)
	def axis_get(self, arg:ObjectAxisId) -> ObjectAxis:
		return self.invoke(0, 'axis_get', arg, ObjectAxis)
	def axis_inherit(self, arg:ObjectAxisId) -> bool:
		return self.invoke(0, 'axis_inherit', arg, None)
	def axis_set(self, arg:ObjectAxisParams) -> bool:
		return self.invoke(0, 'axis_set', arg, None)
	def axis_set_calculated(self, arg:ObjectId) -> ObjectAxis:
		return self.invoke(0, 'axis_set_calculated', arg, ObjectAxis)
	def billet_count_get(self) -> Count:
		return self.invoke(0, 'billet_count_get', None, Count)
	def billet_count_set(self, arg:Count) -> bool:
		return self.invoke(0, 'billet_count_set', arg, None)
	def billet_get_current(self) -> ItemId:
		return self.invoke(0, 'billet_get_current', None, ItemId)
	def billet_parameter_get(self, arg:BilletParameter) -> NullableRealValue:
		return self.invoke(0, 'billet_parameter_get', arg, NullableRealValue)
	def billet_parameter_set(self, arg:BilletParameter) -> bool:
		return self.invoke(0, 'billet_parameter_set', arg, None)
	def billet_set_current(self, arg:ItemId) -> bool:
		return self.invoke(0, 'billet_set_current', arg, None)
	def blow_count_get(self) -> Count:
		return self.invoke(0, 'blow_count_get', None, Count)
	def blow_count_set(self, arg:Count) -> bool:
		return self.invoke(0, 'blow_count_set', arg, None)
	def blow_get_current(self) -> ItemId:
		return self.invoke(0, 'blow_get_current', None, ItemId)
	def blow_parameter_get(self, arg:BlowParameter) -> NullableRealValue:
		return self.invoke(0, 'blow_parameter_get', arg, NullableRealValue)
	def blow_parameter_set(self, arg:BlowParameter) -> bool:
		return self.invoke(0, 'blow_parameter_set', arg, None)
	def blow_set_current(self, arg:ItemId) -> bool:
		return self.invoke(0, 'blow_set_current', arg, None)
	def bound_cond_create(self, arg:BoundCondParams) -> ItemId:
		return self.invoke(0, 'bound_cond_create', arg, ItemId)
	def bound_cond_delete(self, arg:ItemId) -> bool:
		return self.invoke(0, 'bound_cond_delete', arg, None)
	def bound_cond_set_body(self, arg:ShapeBody) -> bool:
		return self.invoke(0, 'bound_cond_set_body', arg, None)
	def bound_cond_set_brick(self, arg:ShapeBrick) -> bool:
		return self.invoke(0, 'bound_cond_set_brick', arg, None)
	def bound_cond_set_circle(self, arg:ShapeCircle) -> bool:
		return self.invoke(0, 'bound_cond_set_circle', arg, None)
	def bound_cond_set_cone(self, arg:ShapeCone) -> bool:
		return self.invoke(0, 'bound_cond_set_cone', arg, None)
	def bound_cond_set_cylinder(self, arg:ShapeCylinder) -> bool:
		return self.invoke(0, 'bound_cond_set_cylinder', arg, None)
	def bound_cond_set_rect(self, arg:ShapeRect) -> bool:
		return self.invoke(0, 'bound_cond_set_rect', arg, None)
	def bound_cond_set_sphere(self, arg:ShapeSphere) -> bool:
		return self.invoke(0, 'bound_cond_set_sphere', arg, None)
	def bound_cond_set_sprayer_polar_array(self, arg:ShapeSprayerPolarArray) -> bool:
		return self.invoke(0, 'bound_cond_set_sprayer_polar_array', arg, None)
	def bound_cond_set_sprayer_polar_array_db(self, arg:ShapeSprayerPolarArrayDB) -> bool:
		return self.invoke(0, 'bound_cond_set_sprayer_polar_array_db', arg, None)
	def bound_cond_set_sprayer_rect_array(self, arg:ShapeSprayerRectArray) -> bool:
		return self.invoke(0, 'bound_cond_set_sprayer_rect_array', arg, None)
	def bound_cond_set_sprayer_rect_array_db(self, arg:ShapeSprayerRectArrayDB) -> bool:
		return self.invoke(0, 'bound_cond_set_sprayer_rect_array_db', arg, None)
	def bound_cond_set_surface_by_color(self, arg:ShapeSurfaceByColor) -> bool:
		return self.invoke(0, 'bound_cond_set_surface_by_color', arg, None)
	def bound_cond_type(self, arg:ItemId) -> BoundCondType:
		return self.invoke(0, 'bound_cond_type', arg, BoundCondType)
	def calculate_tools(self, arg:ObjectList) -> SimulationResult:
		return self.invoke(0, 'calculate_tools', arg, SimulationResult)
	def calculate_tools_coupled(self) -> SimulationResult:
		return self.invoke(0, 'calculate_tools_coupled', None, SimulationResult)
	def chart_get(self, arg:ChartId) -> Chart:
		return self.invoke(0, 'chart_get', arg, Chart)
	def chart_value_get(self, arg:ChartValueId) -> ChartValue:
		return self.invoke(0, 'chart_value_get', arg, ChartValue)
	def client_server_get(self) -> WebAddress:
		return self.invoke(0, 'client_server_get', None, WebAddress)
	def client_server_set(self, arg:WebAddress) -> bool:
		return self.invoke(0, 'client_server_set', arg, None)
	def client_server_test_connection(self, arg:WebAddress) -> ExecutionStatus:
		return self.invoke(0, 'client_server_test_connection', arg, ExecutionStatus)
	def contact_area(self, arg:ObjectId) -> ContactArea:
		return self.invoke(0, 'contact_area', arg, ContactArea)
	def contact_field(self, arg:FieldContact) -> Field:
		return self.invoke(0, 'contact_field', arg, Field)
	def db_arbitrary_drive_get_records(self, arg:DbObjectPath) -> DbArbitraryDriveRecords:
		return self.invoke(0, 'db_arbitrary_drive_get_records', arg, DbArbitraryDriveRecords)
	def db_arbitrary_drive_set_records(self, arg:DbArbitraryDriveRecords) -> bool:
		return self.invoke(0, 'db_arbitrary_drive_set_records', arg, None)
	def db_fetch_items(self, arg:DbFetchParams) -> DbItem:
		return self.invoke(0, 'db_fetch_items', arg, DbItem)
	def db_object_create(self, arg:DbObjectCreationParams) -> bool:
		return self.invoke(0, 'db_object_create', arg, None)
	def db_object_exists(self, arg:PathName) -> BoolValue:
		return self.invoke(0, 'db_object_exists', arg, BoolValue)
	def db_object_export(self, arg:SrcTargetPath) -> bool:
		return self.invoke(0, 'db_object_export', arg, None)
	def db_object_import(self, arg:SrcTargetPath) -> bool:
		return self.invoke(0, 'db_object_import', arg, None)
	def db_object_save(self, arg:SrcTargetPath) -> bool:
		return self.invoke(0, 'db_object_save', arg, None)
	def db_objects_copy_to_project_file(self) -> bool:
		return self.invoke(0, 'db_objects_copy_to_project_file', None, None)
	def db_objects_save_all(self) -> bool:
		return self.invoke(0, 'db_objects_save_all', None, None)
	def db_property_get(self, arg:DbProperty) -> PropertyValue:
		return self.invoke(0, 'db_property_get', arg, PropertyValue)
	def db_property_set(self, arg:DbProperty) -> bool:
		return self.invoke(0, 'db_property_set', arg, None)
	def db_property_table_get(self, arg:DbProperty) -> DbPropertyTable:
		return self.invoke(0, 'db_property_table_get', arg, DbPropertyTable)
	def db_property_table_set(self, arg:DbPropertyTable) -> bool:
		return self.invoke(0, 'db_property_table_set', arg, None)
	def do_batch(self, arg:BatchParams) -> bool:
		return self.invoke(0, 'do_batch', arg, None)
	def domain_create(self, arg:DomainParams) -> ItemId:
		return self.invoke(0, 'domain_create', arg, ItemId)
	def domain_delete(self, arg:ItemId) -> bool:
		return self.invoke(0, 'domain_delete', arg, None)
	def domain_set_body(self, arg:ShapeBody) -> bool:
		return self.invoke(0, 'domain_set_body', arg, None)
	def domain_set_brick(self, arg:ShapeBrick) -> bool:
		return self.invoke(0, 'domain_set_brick', arg, None)
	def domain_set_circle(self, arg:ShapeCircle) -> bool:
		return self.invoke(0, 'domain_set_circle', arg, None)
	def domain_set_cone(self, arg:ShapeCone) -> bool:
		return self.invoke(0, 'domain_set_cone', arg, None)
	def domain_set_cylinder(self, arg:ShapeCylinder) -> bool:
		return self.invoke(0, 'domain_set_cylinder', arg, None)
	def domain_set_rect(self, arg:ShapeRect) -> bool:
		return self.invoke(0, 'domain_set_rect', arg, None)
	def domain_set_sphere(self, arg:ShapeSphere) -> bool:
		return self.invoke(0, 'domain_set_sphere', arg, None)
	def domain_set_surface_by_color(self, arg:ShapeSurfaceByColor) -> bool:
		return self.invoke(0, 'domain_set_surface_by_color', arg, None)
	def domain_type(self, arg:ItemId) -> DomainType:
		return self.invoke(0, 'domain_type', arg, DomainType)
	def dxf_parse_contours(self, arg:FileName) -> Contours:
		return self.invoke(0, 'dxf_parse_contours', arg, Contours)
	def execute_subroutines(self) -> SimulationResult:
		return self.invoke(0, 'execute_subroutines', None, SimulationResult)
	def execute_subroutines_advanced(self, arg:SubroutineCalculationMode) -> SimulationResult:
		return self.invoke(0, 'execute_subroutines_advanced', arg, SimulationResult)
	def execute_tracking(self) -> bool:
		return self.invoke(0, 'execute_tracking', None, None)
	def execute_tracking_advanced(self, arg:TrackingCalculationMode) -> SimulationResult:
		return self.invoke(0, 'execute_tracking_advanced', arg, SimulationResult)
	def export_bearing_contours(self, arg:BearingContoursExport) -> bool:
		return self.invoke(0, 'export_bearing_contours', arg, None)
	def export_field_isolines(self, arg:ExportFieldIsolines) -> bool:
		return self.invoke(0, 'export_field_isolines', arg, None)
	def export_field_isosurface(self, arg:ExportFieldIsosurface) -> bool:
		return self.invoke(0, 'export_field_isosurface', arg, None)
	def export_fields_at_tracking_points(self, arg:ExportFieldsAtTrackingPoints) -> bool:
		return self.invoke(0, 'export_fields_at_tracking_points', arg, None)
	def export_mesh(self, arg:MeshExport) -> bool:
		return self.invoke(0, 'export_mesh', arg, None)
	def export_profile_section(self, arg:ProfileSectionExport) -> bool:
		return self.invoke(0, 'export_profile_section', arg, None)
	def export_records(self, arg:RecordsExport) -> bool:
		return self.invoke(0, 'export_records', arg, None)
	def export_screenshot(self, arg:ExportImage) -> bool:
		return self.invoke(0, 'export_screenshot', arg, None)
	def export_section_mesh(self, arg:ExportSection) -> bool:
		return self.invoke(0, 'export_section_mesh', arg, None)
	def export_video(self, arg:ExportImage) -> bool:
		return self.invoke(0, 'export_video', arg, None)
	def extrusion_trace_count(self) -> Count:
		return self.invoke(0, 'extrusion_trace_count', None, Count)
	def extrusion_trace_get(self, arg:TraceId) -> Trace:
		return self.invoke(0, 'extrusion_trace_get', arg, Trace)
	def field_at_mesh_point(self, arg:FieldAtMeshPoint) -> RealValue:
		return self.invoke(0, 'field_at_mesh_point', arg, RealValue)
	def field_at_mesh_point_vector(self, arg:FieldAtMeshPoint) -> VectorValue:
		return self.invoke(0, 'field_at_mesh_point_vector', arg, VectorValue)
	def field_at_point(self, arg:FieldAtPoint) -> RealValue:
		return self.invoke(0, 'field_at_point', arg, RealValue)
	def field_at_point_vector(self, arg:FieldAtPoint) -> VectorValue:
		return self.invoke(0, 'field_at_point_vector', arg, VectorValue)
	def field_at_tracking_line(self, arg:FieldAtTrackingObject) -> RealValues:
		return self.invoke(0, 'field_at_tracking_line', arg, RealValues)
	def field_at_tracking_line_vector(self, arg:FieldAtTrackingObject) -> VectorValues:
		return self.invoke(0, 'field_at_tracking_line_vector', arg, VectorValues)
	def field_at_tracking_point(self, arg:FieldAtTrackingObject) -> RealValue:
		return self.invoke(0, 'field_at_tracking_point', arg, RealValue)
	def field_at_tracking_point_vector(self, arg:FieldAtTrackingObject) -> VectorValue:
		return self.invoke(0, 'field_at_tracking_point_vector', arg, VectorValue)
	def field_get(self, arg:FieldAtMesh) -> Field:
		return self.invoke(0, 'field_get', arg, Field)
	def field_get_vector(self, arg:FieldAtMesh) -> VectorField:
		return self.invoke(0, 'field_get_vector', arg, VectorField)
	def field_isolines(self, arg:FieldIsosurface) -> IsolineList:
		return self.invoke(0, 'field_isolines', arg, IsolineList)
	def field_isosurface(self, arg:FieldIsosurface) -> IsosurfaceList:
		return self.invoke(0, 'field_isosurface', arg, IsosurfaceList)
	def field_min_max(self, arg:FieldAtMinMax) -> FieldMinMax:
		return self.invoke(0, 'field_min_max', arg, FieldMinMax)
	def field_mode_get(self) -> FieldMode:
		return self.invoke(0, 'field_mode_get', None, FieldMode)
	def field_mode_set(self, arg:FieldMode) -> bool:
		return self.invoke(0, 'field_mode_set', arg, None)
	def field_palette_get(self) -> FieldPalette:
		return self.invoke(0, 'field_palette_get', None, FieldPalette)
	def field_palette_set(self, arg:FieldPalette) -> bool:
		return self.invoke(0, 'field_palette_set', arg, None)
	def field_stat(self, arg:FieldStatAtMesh) -> FieldStat:
		return self.invoke(0, 'field_stat', arg, FieldStat)
	def field_stat_at_section(self, arg:FieldStatAtSection) -> FieldStat:
		return self.invoke(0, 'field_stat_at_section', arg, FieldStat)
	def file_dialog(self, arg:FileDlg) -> StringList:
		return self.invoke(0, 'file_dialog', arg, StringList)
	def geometry_convert_to_3d(self, arg:ConvertTo3d) -> bool:
		return self.invoke(0, 'geometry_convert_to_3d', arg, None)
	def geometry_create_brick(self, arg:BrickObjectParams) -> bool:
		return self.invoke(0, 'geometry_create_brick', arg, None)
	def geometry_create_rect(self, arg:RectObjectParams) -> bool:
		return self.invoke(0, 'geometry_create_rect', arg, None)
	def geometry_create_sphere(self, arg:SphereObjectParams) -> bool:
		return self.invoke(0, 'geometry_create_sphere', arg, None)
	def geometry_create_tube(self, arg:TubeObjectParams) -> bool:
		return self.invoke(0, 'geometry_create_tube', arg, None)
	def geometry_load(self, arg:FileName) -> ObjectList:
		return self.invoke(0, 'geometry_load', arg, ObjectList)
	def geometry_load_extruded_object(self, arg:ExtrudedObject) -> bool:
		return self.invoke(0, 'geometry_load_extruded_object', arg, None)
	def geometry_load_revolved_object(self, arg:RevolvedObject) -> bool:
		return self.invoke(0, 'geometry_load_revolved_object', arg, None)
	def geometry_load_single_object(self, arg:FileObject) -> bool:
		return self.invoke(0, 'geometry_load_single_object', arg, None)
	def gravity_positioning(self, arg:GravityPositioning) -> bool:
		return self.invoke(0, 'gravity_positioning', arg, None)
	def is_windowless_mode(self) -> BoolValue:
		return self.invoke(0, 'is_windowless_mode', None, BoolValue)
	def key_names_get(self) -> KeyNames:
		return self.invoke(0, 'key_names_get', None, KeyNames)
	def key_send(self, arg:SendKey) -> bool:
		return self.invoke(0, 'key_send', arg, None)
	def language_get(self) -> QFormLang:
		return self.invoke(0, 'language_get', None, QFormLang)
	def language_set(self, arg:QFormLang) -> bool:
		return self.invoke(0, 'language_set', arg, None)
	def log_begin(self, arg:LogParams) -> bool:
		return self.invoke(0, 'log_begin', arg, None)
	def log_save(self, arg:LogFile) -> bool:
		return self.invoke(0, 'log_save', arg, None)
	def mesh_apex_get(self, arg:MeshApexId) -> MeshApex:
		return self.invoke(0, 'mesh_apex_get', arg, MeshApex)
	def mesh_cubics_get(self, arg:MeshObjectId) -> MeshCubics:
		return self.invoke(0, 'mesh_cubics_get', arg, MeshCubics)
	def mesh_edge_get(self, arg:MeshEdgeId) -> MeshEdge:
		return self.invoke(0, 'mesh_edge_get', arg, MeshEdge)
	def mesh_face_get(self, arg:MeshFaceId) -> MeshFace:
		return self.invoke(0, 'mesh_face_get', arg, MeshFace)
	def mesh_face_types_get(self, arg:MeshObjectId) -> FaceTypes:
		return self.invoke(0, 'mesh_face_types_get', arg, FaceTypes)
	def mesh_lap_points_get(self, arg:MeshObjectId) -> MeshCoords:
		return self.invoke(0, 'mesh_lap_points_get', arg, MeshCoords)
	def mesh_node_owners_get(self, arg:MeshObjectId) -> MeshNodeOwners:
		return self.invoke(0, 'mesh_node_owners_get', arg, MeshNodeOwners)
	def mesh_nodes_get(self, arg:MeshObjectId) -> MeshCoords:
		return self.invoke(0, 'mesh_nodes_get', arg, MeshCoords)
	def mesh_point_get(self, arg:ObjectPoint) -> MeshPoint:
		return self.invoke(0, 'mesh_point_get', arg, MeshPoint)
	def mesh_properties_get(self, arg:MeshObjectId) -> MeshProperties:
		return self.invoke(0, 'mesh_properties_get', arg, MeshProperties)
	def mesh_quadrangles_get(self, arg:MeshObjectId) -> MeshQuadrangles:
		return self.invoke(0, 'mesh_quadrangles_get', arg, MeshQuadrangles)
	def mesh_thetrahedrons_get(self, arg:MeshObjectId) -> MeshTetrahedrons:
		return self.invoke(0, 'mesh_thetrahedrons_get', arg, MeshTetrahedrons)
	def mesh_triangles_get(self, arg:MeshObjectId) -> MeshTriangles:
		return self.invoke(0, 'mesh_triangles_get', arg, MeshTriangles)
	def mouse_click(self, arg:MouseClick) -> bool:
		return self.invoke(0, 'mouse_click', arg, None)
	def mouse_click_capture(self) -> MouseClick:
		return self.invoke(0, 'mouse_click_capture', None, MouseClick)
	def mouse_pos_get(self) -> MousePos:
		return self.invoke(0, 'mouse_pos_get', None, MousePos)
	def mouse_pos_set(self, arg:MousePos) -> bool:
		return self.invoke(0, 'mouse_pos_set', arg, None)
	def msg_box(self, arg:MsgBox) -> PressedDialogButton:
		return self.invoke(0, 'msg_box', arg, PressedDialogButton)
	def object_apply_transform(self, arg:ObjectTransform) -> bool:
		return self.invoke(0, 'object_apply_transform', arg, None)
	def object_axes_get(self, arg:ObjectId) -> ItemList:
		return self.invoke(0, 'object_axes_get', arg, ItemList)
	def object_bound_conds_get(self, arg:ObjectId) -> ItemList:
		return self.invoke(0, 'object_bound_conds_get', arg, ItemList)
	def object_contact(self, arg:ObjectContact) -> RealValue:
		return self.invoke(0, 'object_contact', arg, RealValue)
	def object_copy(self, arg:ObjectConvert) -> bool:
		return self.invoke(0, 'object_copy', arg, None)
	def object_delete(self, arg:ObjectId) -> bool:
		return self.invoke(0, 'object_delete', arg, None)
	def object_display_mode_get(self, arg:DisplayMode) -> BoolValue:
		return self.invoke(0, 'object_display_mode_get', arg, BoolValue)
	def object_display_mode_set(self, arg:DisplayMode) -> bool:
		return self.invoke(0, 'object_display_mode_set', arg, None)
	def object_displayed_name(self, arg:ObjectId) -> ObjectName:
		return self.invoke(0, 'object_displayed_name', arg, ObjectName)
	def object_domains_get(self, arg:ObjectId) -> ItemList:
		return self.invoke(0, 'object_domains_get', arg, ItemList)
	def object_exists(self, arg:ObjectId) -> BoolValue:
		return self.invoke(0, 'object_exists', arg, BoolValue)
	def object_find_by_color(self, arg:FindByColor) -> ObjectId:
		return self.invoke(0, 'object_find_by_color', arg, ObjectId)
	def object_find_by_surface_point(self, arg:FindByPoint) -> ObjectId:
		return self.invoke(0, 'object_find_by_surface_point', arg, ObjectId)
	def object_inherit(self, arg:ObjectId) -> bool:
		return self.invoke(0, 'object_inherit', arg, None)
	def object_is_inherited(self, arg:ObjectId) -> BoolValue:
		return self.invoke(0, 'object_is_inherited', arg, BoolValue)
	def object_move(self, arg:ObjectMove) -> bool:
		return self.invoke(0, 'object_move', arg, None)
	def object_move_along_axis(self, arg:ObjectMoveAxis) -> bool:
		return self.invoke(0, 'object_move_along_axis', arg, None)
	def object_rotate(self, arg:ObjectRotate) -> bool:
		return self.invoke(0, 'object_rotate', arg, None)
	def object_rotate_around_axis(self, arg:ObjectRotateAxis) -> bool:
		return self.invoke(0, 'object_rotate_around_axis', arg, None)
	def object_set_type_by_color(self, arg:TypeSetByColor) -> bool:
		return self.invoke(0, 'object_set_type_by_color', arg, None)
	def object_set_type_by_surface_point(self, arg:TypeSetByPoint) -> bool:
		return self.invoke(0, 'object_set_type_by_surface_point', arg, None)
	def object_type_set(self, arg:ObjectConvert) -> bool:
		return self.invoke(0, 'object_type_set', arg, None)
	def object_type_set_in_direction(self, arg:ObjectsInDirection) -> bool:
		return self.invoke(0, 'object_type_set_in_direction', arg, None)
	def objects_find_by_color(self, arg:FindByColor) -> ObjectList:
		return self.invoke(0, 'objects_find_by_color', arg, ObjectList)
	def objects_get_in_direction(self, arg:PickDirection) -> ObjectList:
		return self.invoke(0, 'objects_get_in_direction', arg, ObjectList)
	def operation_chains_get(self, arg:ItemId) -> ItemList:
		return self.invoke(0, 'operation_chains_get', arg, ItemList)
	def operation_check(self, arg:OptionalItemId) -> OperationChecks:
		return self.invoke(0, 'operation_check', arg, OperationChecks)
	def operation_copy(self, arg:OperationCopy) -> ItemId:
		return self.invoke(0, 'operation_copy', arg, ItemId)
	def operation_copy_from_parent(self, arg:OperationCopyFromParent) -> bool:
		return self.invoke(0, 'operation_copy_from_parent', arg, None)
	def operation_create(self, arg:OperationParams) -> ItemId:
		return self.invoke(0, 'operation_create', arg, ItemId)
	def operation_cut(self, arg:ItemId) -> bool:
		return self.invoke(0, 'operation_cut', arg, None)
	def operation_delete(self, arg:ItemId) -> bool:
		return self.invoke(0, 'operation_delete', arg, None)
	def operation_exists(self, arg:ItemId) -> BoolValue:
		return self.invoke(0, 'operation_exists', arg, BoolValue)
	def operation_get(self, arg:ItemId) -> Operation:
		return self.invoke(0, 'operation_get', arg, Operation)
	def operation_get_by_uid(self, arg:ItemId) -> ItemId:
		return self.invoke(0, 'operation_get_by_uid', arg, ItemId)
	def operation_get_current(self) -> ItemId:
		return self.invoke(0, 'operation_get_current', None, ItemId)
	def operation_graph_get(self, arg:ItemId) -> OperationGraph:
		return self.invoke(0, 'operation_graph_get', arg, OperationGraph)
	def operation_insert(self, arg:OperationInsert) -> ItemId:
		return self.invoke(0, 'operation_insert', arg, ItemId)
	def operation_process_get(self, arg:ItemId) -> ItemId:
		return self.invoke(0, 'operation_process_get', arg, ItemId)
	def operation_set_current(self, arg:ItemId) -> bool:
		return self.invoke(0, 'operation_set_current', arg, None)
	def operation_set_next_in_chain(self) -> ItemId:
		return self.invoke(0, 'operation_set_next_in_chain', None, ItemId)
	def operation_set_prev_in_chain(self) -> ItemId:
		return self.invoke(0, 'operation_set_prev_in_chain', None, ItemId)
	def operation_template_set(self, arg:DbObjectPath) -> bool:
		return self.invoke(0, 'operation_template_set', arg, None)
	def operation_tree(self) -> Operation:
		return self.invoke(0, 'operation_tree', None, Operation)
	def operation_uid(self, arg:OptionalItemId) -> ItemId:
		return self.invoke(0, 'operation_uid', arg, ItemId)
	def panel_positions_get(self) -> PanelPositions:
		return self.invoke(0, 'panel_positions_get', None, PanelPositions)
	def panel_sizes_get(self) -> PanelSizes:
		return self.invoke(0, 'panel_sizes_get', None, PanelSizes)
	def panel_sizes_set(self, arg:PanelSizes) -> bool:
		return self.invoke(0, 'panel_sizes_set', arg, None)
	def print(self, arg:TraceMsg) -> bool:
		return self.invoke(0, 'print', arg, None)
	def process_chain_get_current(self) -> ItemId:
		return self.invoke(0, 'process_chain_get_current', None, ItemId)
	def process_chain_get_current_operations(self) -> ItemList:
		return self.invoke(0, 'process_chain_get_current_operations', None, ItemList)
	def process_chain_get_operations(self, arg:ItemId) -> ItemList:
		return self.invoke(0, 'process_chain_get_operations', arg, ItemList)
	def process_chain_set_current(self, arg:ItemId) -> bool:
		return self.invoke(0, 'process_chain_set_current', arg, None)
	def processes_get(self) -> ItemList:
		return self.invoke(0, 'processes_get', None, ItemList)
	def project_ask_save(self) -> BoolValue:
		return self.invoke(0, 'project_ask_save', None, BoolValue)
	def project_file_get(self) -> FileName:
		return self.invoke(0, 'project_file_get', None, FileName)
	def project_new(self) -> bool:
		return self.invoke(0, 'project_new', None, None)
	def project_open(self, arg:FileName) -> bool:
		return self.invoke(0, 'project_open', arg, None)
	def project_open_as_copy(self, arg:ProjectOpenAsCopy) -> bool:
		return self.invoke(0, 'project_open_as_copy', arg, None)
	def project_open_or_create(self, arg:FileName) -> bool:
		return self.invoke(0, 'project_open_or_create', arg, None)
	def project_path_get(self) -> FileName:
		return self.invoke(0, 'project_path_get', None, FileName)
	def project_save(self) -> bool:
		return self.invoke(0, 'project_save', None, None)
	def project_save_as(self, arg:FileName) -> bool:
		return self.invoke(0, 'project_save_as', arg, None)
	def project_save_as_template(self, arg:PathName) -> bool:
		return self.invoke(0, 'project_save_as_template', arg, None)
	def property_get(self, arg:Property) -> PropertyValue:
		return self.invoke(0, 'property_get', arg, PropertyValue)
	def property_get_array_of_real(self, arg:PropertyPath) -> ArrayOfReal:
		return self.invoke(0, 'property_get_array_of_real', arg, ArrayOfReal)
	def property_get_object(self, arg:PropertyPath) -> ObjectId:
		return self.invoke(0, 'property_get_object', arg, ObjectId)
	def property_set(self, arg:Property) -> bool:
		return self.invoke(0, 'property_set', arg, None)
	def property_set_array_of_real(self, arg:PropertyArrayOfReal) -> bool:
		return self.invoke(0, 'property_set_array_of_real', arg, None)
	def property_set_object(self, arg:ObjectIdProperty) -> bool:
		return self.invoke(0, 'property_set_object', arg, None)
	def qform_attach_to(self, arg:SessionId) -> bool:
		return self.invoke(-6, 'qform_attach_to', arg, None)
	def qform_process_id(self) -> ProcessId:
		return self.invoke(0, 'qform_process_id', None, ProcessId)
	def qform_reconnect(self) -> bool:
		return self.invoke(-4, 'qform_reconnect', None, None)
	def qform_version(self) -> QFormVer:
		return self.invoke(0, 'qform_version', None, QFormVer)
	def qform_window_id(self) -> WindowId:
		return self.invoke(0, 'qform_window_id', None, WindowId)
	def qform_window_pos_get(self) -> WindowPosition:
		return self.invoke(0, 'qform_window_pos_get', None, WindowPosition)
	def qform_window_pos_set(self, arg:WindowPosition) -> bool:
		return self.invoke(0, 'qform_window_pos_set', arg, None)
	def record_get(self) -> Record:
		return self.invoke(0, 'record_get', None, Record)
	def record_get_last(self) -> Record:
		return self.invoke(0, 'record_get_last', None, Record)
	def record_set(self, arg:Record) -> bool:
		return self.invoke(0, 'record_set', arg, None)
	def results_truncate(self, arg:Record) -> bool:
		return self.invoke(0, 'results_truncate', arg, None)
	def section_mesh_get(self, arg:SectionMeshPlane) -> SectionMeshList:
		return self.invoke(0, 'section_mesh_get', arg, SectionMeshList)
	def section_plane_create_3p(self, arg:SectionPlane3P) -> ItemId:
		return self.invoke(0, 'section_plane_create_3p', arg, ItemId)
	def section_plane_create_pn(self, arg:SectionPlanePN) -> ItemId:
		return self.invoke(0, 'section_plane_create_pn', arg, ItemId)
	def section_plane_delete(self, arg:ItemId) -> bool:
		return self.invoke(0, 'section_plane_delete', arg, None)
	def section_plane_get(self, arg:ItemId) -> SectionPlane:
		return self.invoke(0, 'section_plane_get', arg, SectionPlane)
	def section_planes_get(self) -> ItemList:
		return self.invoke(0, 'section_planes_get', None, ItemList)
	def selected_object_get(self) -> ObjectId:
		return self.invoke(0, 'selected_object_get', None, ObjectId)
	def session_id(self) -> ItemId:
		return self.invoke(0, 'session_id', None, ItemId)
	def session_info(self) -> Session:
		return self.invoke(0, 'session_info', None, Session)
	def session_info_by_id(self, arg:SessionId) -> Session:
		return self.invoke(-7, 'session_info_by_id', arg, Session)
	def session_list(self) -> SessionList:
		return self.invoke(-5, 'session_list', None, SessionList)
	def session_max_count(self) -> Count:
		return self.invoke(0, 'session_max_count', None, Count)
	def simulation_objects_get(self) -> ObjectList:
		return self.invoke(0, 'simulation_objects_get', None, ObjectList)
	def sleep(self, arg:SleepTime) -> bool:
		return self.invoke(0, 'sleep', arg, None)
	def start_simulation(self) -> MainSimulationResult:
		return self.invoke(0, 'start_simulation', None, MainSimulationResult)
	def start_simulation_advanced(self, arg:SimulationParams) -> MainSimulationResult:
		return self.invoke(0, 'start_simulation_advanced', arg, MainSimulationResult)
	def state_blow(self, arg:OptionalGlobalItemId) -> SimulationState:
		return self.invoke(0, 'state_blow', arg, SimulationState)
	def state_extrusion(self, arg:SystemStateId) -> ExtrusionState:
		return self.invoke(0, 'state_extrusion', arg, ExtrusionState)
	def state_mesh(self, arg:MeshStateId) -> MeshState:
		return self.invoke(0, 'state_mesh', arg, MeshState)
	def state_operation(self, arg:OptionalItemId) -> SimulationState:
		return self.invoke(0, 'state_operation', arg, SimulationState)
	def state_process_chain(self, arg:OptionalItemId) -> SimulationState:
		return self.invoke(0, 'state_process_chain', arg, SimulationState)
	def state_system(self, arg:SystemStateId) -> SystemState:
		return self.invoke(0, 'state_system', arg, SystemState)
	def state_tool(self, arg:ToolStateId) -> ToolState:
		return self.invoke(0, 'state_tool', arg, ToolState)
	def state_workpiece(self, arg:WorkpieceStateId) -> WorkpieceState:
		return self.invoke(0, 'state_workpiece', arg, WorkpieceState)
	def stop_cond_create(self, arg:StopCondParams) -> ItemId:
		return self.invoke(0, 'stop_cond_create', arg, ItemId)
	def stop_cond_create_distance(self, arg:StopCondDistance) -> ItemId:
		return self.invoke(0, 'stop_cond_create_distance', arg, ItemId)
	def stop_cond_create_final_pos(self, arg:StopCondFinPos) -> ItemId:
		return self.invoke(0, 'stop_cond_create_final_pos', arg, ItemId)
	def stop_cond_create_max_load(self, arg:StopCondMaxLoad) -> ItemId:
		return self.invoke(0, 'stop_cond_create_max_load', arg, ItemId)
	def stop_cond_create_rotation(self, arg:StopCondRotation) -> ItemId:
		return self.invoke(0, 'stop_cond_create_rotation', arg, ItemId)
	def stop_cond_create_stroke(self, arg:StopCondStroke) -> ItemId:
		return self.invoke(0, 'stop_cond_create_stroke', arg, ItemId)
	def stop_cond_create_time(self, arg:StopCondTime) -> ItemId:
		return self.invoke(0, 'stop_cond_create_time', arg, ItemId)
	def stop_cond_delete(self, arg:ItemId) -> bool:
		return self.invoke(0, 'stop_cond_delete', arg, None)
	def stop_cond_type(self, arg:ItemId) -> StopCond:
		return self.invoke(0, 'stop_cond_type', arg, StopCond)
	def stop_condition_create_field_value(self, arg:StopCondFieldValue) -> ItemId:
		return self.invoke(0, 'stop_condition_create_field_value', arg, ItemId)
	def stop_conds_get(self) -> ItemList:
		return self.invoke(0, 'stop_conds_get', None, ItemList)
	def subroutine_create(self, arg:SubroutineCreate) -> ItemId:
		return self.invoke(0, 'subroutine_create', arg, ItemId)
	def subroutine_delete(self, arg:ItemId) -> bool:
		return self.invoke(0, 'subroutine_delete', arg, None)
	def subroutine_get(self, arg:ItemId) -> Subroutine:
		return self.invoke(0, 'subroutine_get', arg, Subroutine)
	def subroutine_parameter_get(self, arg:SubroutineParameter) -> NullableRealValue:
		return self.invoke(0, 'subroutine_parameter_get', arg, NullableRealValue)
	def subroutine_parameter_set(self, arg:SubroutineParameter) -> bool:
		return self.invoke(0, 'subroutine_parameter_set', arg, None)
	def subroutines_get(self) -> ItemList:
		return self.invoke(0, 'subroutines_get', None, ItemList)
	def sym_plane_create(self, arg:SymPlaneParams) -> ItemId:
		return self.invoke(0, 'sym_plane_create', arg, ItemId)
	def sym_plane_create_by_close_point(self, arg:SymPlaneByPoint) -> ItemId:
		return self.invoke(0, 'sym_plane_create_by_close_point', arg, ItemId)
	def sym_plane_create_by_face_color(self, arg:SymPlaneByColor) -> ItemId:
		return self.invoke(0, 'sym_plane_create_by_face_color', arg, ItemId)
	def sym_plane_delete(self, arg:ItemId) -> bool:
		return self.invoke(0, 'sym_plane_delete', arg, None)
	def sym_plane_get(self, arg:ItemId) -> UnitVector:
		return self.invoke(0, 'sym_plane_get', arg, UnitVector)
	def sym_planes_create_auto(self) -> ItemList:
		return self.invoke(0, 'sym_planes_create_auto', None, ItemList)
	def sym_planes_get(self) -> ItemList:
		return self.invoke(0, 'sym_planes_get', None, ItemList)
	def system_of_units_get(self) -> UnitSystem:
		return self.invoke(0, 'system_of_units_get', None, UnitSystem)
	def system_of_units_set(self, arg:UnitSystem) -> bool:
		return self.invoke(0, 'system_of_units_set', arg, None)
	def tracking_contour_create(self, arg:NewTrackingContour) -> ItemId:
		return self.invoke(0, 'tracking_contour_create', arg, ItemId)
	def tracking_contours_create(self, arg:OptionalObjectItemId) -> ItemId:
		return self.invoke(0, 'tracking_contours_create', arg, ItemId)
	def tracking_group_create(self, arg:TrackingGroup) -> ItemId:
		return self.invoke(0, 'tracking_group_create', arg, ItemId)
	def tracking_line_create(self, arg:TrackingLineParams) -> ItemId:
		return self.invoke(0, 'tracking_line_create', arg, ItemId)
	def tracking_line_get(self, arg:GlobalItemId) -> TrackingLine:
		return self.invoke(0, 'tracking_line_get', arg, TrackingLine)
	def tracking_lines_get(self) -> ItemList:
		return self.invoke(0, 'tracking_lines_get', None, ItemList)
	def tracking_lines_get_for_chain(self) -> GlobalItemList:
		return self.invoke(0, 'tracking_lines_get_for_chain', None, GlobalItemList)
	def tracking_point_create(self, arg:TrackingPointParams) -> ItemId:
		return self.invoke(0, 'tracking_point_create', arg, ItemId)
	def tracking_point_get(self, arg:GlobalItemId) -> MeshPoint:
		return self.invoke(0, 'tracking_point_get', arg, MeshPoint)
	def tracking_points_get(self) -> ItemList:
		return self.invoke(0, 'tracking_points_get', None, ItemList)
	def tracking_points_get_for_chain(self) -> GlobalItemList:
		return self.invoke(0, 'tracking_points_get_for_chain', None, GlobalItemList)
	def view_back(self) -> bool:
		return self.invoke(0, 'view_back', None, None)
	def view_bottom(self) -> bool:
		return self.invoke(0, 'view_bottom', None, None)
	def view_front(self) -> bool:
		return self.invoke(0, 'view_front', None, None)
	def view_get(self) -> View:
		return self.invoke(0, 'view_get', None, View)
	def view_left(self) -> bool:
		return self.invoke(0, 'view_left', None, None)
	def view_on_bottom_90(self) -> bool:
		return self.invoke(0, 'view_on_bottom_90', None, None)
	def view_on_top_90(self) -> bool:
		return self.invoke(0, 'view_on_top_90', None, None)
	def view_option_get(self, arg:ViewOption) -> BoolValue:
		return self.invoke(0, 'view_option_get', arg, BoolValue)
	def view_option_set(self, arg:ViewOption) -> bool:
		return self.invoke(0, 'view_option_set', arg, None)
	def view_right(self) -> bool:
		return self.invoke(0, 'view_right', None, None)
	def view_set(self, arg:View) -> bool:
		return self.invoke(0, 'view_set', arg, None)
	def view_top(self) -> bool:
		return self.invoke(0, 'view_top', None, None)
	def work_dir_get(self) -> PathName:
		return self.invoke(0, 'work_dir_get', None, PathName)
	def work_dir_set(self, arg:PathName) -> bool:
		return self.invoke(0, 'work_dir_set', arg, None)
	def zoom_to_fit(self) -> bool:
		return self.invoke(0, 'zoom_to_fit', None, None)

