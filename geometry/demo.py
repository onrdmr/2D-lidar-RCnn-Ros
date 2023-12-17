#!/usr/bin/env python


r"""
refactor:

range(1,NbSol+1) -> range(NbSol)
"a string"+`i` -> "a string %s" % (i) 


TODO:

    *curves3d_from_points doesnt work well; missing a spline
    *make_text is a TOTAL joke... completely unacceptable...
    *in the bspline example, the conversion from a list to TColgp_* fails somehow...
        usually the call to _Tcol_* seems to work, not here though...
    *630: circle not rendered
    *pipes gives terrible results... 
    *1109 converting list to TCol* flips...

"""

import time
import sys

from OCC.gp import *
from OCC.Geom2d import *
from OCC.Geom2dAdaptor import *
from OCC.Geom2dAPI import *
from OCC.GCPnts import *

import OCC._GccEnt
import OCC.Precision
import OCC.Geom2dConvert
import OCC.Convert
import OCC.Precision
import OCC.gp

from OCC.Geom import *
from OCC.GeomAPI import *
from OCC.IntAna import *
from OCC.GC import *
from OCC.GCE2d import *
from OCC.Geom2dConvert import *
from OCC.TopAbs import *
from OCC.GccEnt import *
from OCC.gce import *
from OCC.GccAna import *
from OCC.Quantity import *
from OCC.GeomConvert import *
from OCC.TColGeom import *
from OCC.BRepBuilderAPI import *
from OCC.Graphic2d import *
from OCC.TCollection import *
from OCC.Graphic3d import *
from OCC.BRepPrimAPI import *
from OCC.AIS import *
from OCC.Prs3d import *
from OCC.TColgp import * 
from OCC.GeomFill import *

from OCC.Display.SimpleGui import *

display, start_display, add_menu, add_function_to_menu = init_display('wx')

TEXT_HEIGHT = 15

# ===============================================================================
# Utility functions
# ===============================================================================


def _Tcol_dim_1(li, _type):
    r"""function factory for 1-dimensional TCol* types"""
    pts = _type(0, len(li)-1)
    for n,i in enumerate(li):
        pts.SetValue(n,i)
    return pts


def _Tcol_dim_2(li, _type):
    r"""function factory for 2-dimensional TCol* types"""
    length_nested = len(li[0])-1
    pts = _type(0, len(li)-1, 0, length_nested)
    return pts
    # for n1, i in enumerate(li):
    #     for n2, j in enumerate(i):
    #         pts.SetValue(n1, n2, j)
    # return pts


def point_list_to_TColgp_Array1OfPnt(li):
    pts = TColgp_Array1OfPnt(0, len(li)-1)
    for n, i in enumerate(li):
        pts.SetValue(n, i)
    return pts


def point2d_list_to_TColgp_Array1OfPnt2d(li):
    return _Tcol_dim_1(li, TColgp_Array1OfPnt2d)


def make_text(string, pnt, height):
    r"""
    render a bunch of text
    @param string: string to be rendered
    @param pnt:    location of the string
    @param myGroup:OCC.Graphic3d.Graphic3d_Group instance
    @param height: max height
    """
    global display
    # returns a Handle_Visual3d_ViewManager instance
    # the only thing is that you need the Visual3d class to make this work well
    # now we have to make a presentation for a stupid sphere as a workaround to get to the object
# ===============================================================================
#     The reason for recreating is that myGroup is gone after an EraseAll call
# ===============================================================================
#     stupid_sphere = BRepPrimAPI_MakeSphere(1, 1, 1, 1)
#     prs_sphere = AIS_Shape(stupid_sphere.Shape())
#     ais_context = display.GetContext().GetObject()  # display.GetContext() returns a Handle_AIS_InteractiveContext
#     prsMgr = ais_context.MainPrsMgr().GetObject()
#     ais_context.Display(prs_sphere.GetHandle(), True)
    # aPresentation = prsMgr.CastPresentation(prs_sphere.GetHandle()).GetObject()
    # global myGroup
    # myGroup = Prs3d_Root_CurrentGroup(aPresentation.Presentation()).GetObject()
# ===============================================================================
#    FINE
# ===============================================================================
#     _string = TCollection_ExtendedString(string)
#     if isinstance(pnt, OCC.gp.gp_Pnt2d):
#         _vertex = Graphic3d_Vertex(pnt.X(), pnt.Y(), 0)
#     else:
#         _vertex = Graphic3d_Vertex(pnt.X(), pnt.Y(), pnt.Z())
#     myGroup.Text(_string, _vertex, height)
    display.DisplayMessage(point=pnt, text_to_write=string, height=height, message_color=(0, 0, 0))


def make_edge2d(shape):
    spline = BRepBuilderAPI_MakeEdge2d(shape)
    spline.Build()
    return spline.Shape()


def make_edge(shape):
    spline = BRepBuilderAPI_MakeEdge(shape)
    spline.Build()
    return spline.Shape()


def make_vertex(pnt):
    if isinstance(pnt, OCC.gp.gp_Pnt2d):
        vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(pnt.X(), pnt.Y(), 0))
    else: 
        vertex = BRepBuilderAPI_MakeVertex(pnt)
    vertex.Build()
    return vertex.Shape()


def make_face(shape):
    tol_degen = 1e-6
    face = BRepBuilderAPI_MakeFace(shape, tol_degen)
    face.Build()
    return face.Shape()

# ===============================================================================
# Examples
# ===============================================================================


def point_from_curve( event=None ):
    r"""
    @param display: instance of wxViewer3d
    """
    global myGroup
    display.EraseAll()
    radius, abscissa = 5., 3.                              
    C = Geom2d_Circle(gp_OX2d(), radius, True)
    GAC = Geom2dAdaptor_Curve(C.GetHandle())
    UA = GCPnts_UniformAbscissa(GAC, abscissa)
    
    aSequence = []
    if UA.IsDone():       
        N = UA.NbPoints()
        for count in range(1, N + 1):
            P = OCC.gp.gp_Pnt2d()
            C.D0(UA.Parameter(count), P)
            Parameter = UA.Parameter(count)
            aSequence.append(P)

    Abscissa = UA.Abscissa()
    assert abscissa == Abscissa, 'abscissas dont match up...'
    
    # convert analytic to bspline
    display.DisplayShape(make_edge2d(C.Circ2d()), update=True)
    
    i = 0
    for P in aSequence:
        i += 1
        pstring = 'P' + i + ': Parameter :' + UA.Parameter( i )
        YOffset = - 0.3
        YOffset += 0.2 * (i == 1)
        YOffset += 0.4 * (i == 4)
        YOffset += - 0.3 * (i == len(aSequence))
        pnt = OCC.gp.gp_Pnt(P.X(), P.Y(), 0)
        display.DisplayShape(make_vertex(pnt))
        make_text(pstring, pnt, TEXT_HEIGHT)
        
    print ('completed point_from_curve, moving on...')

def project_point_on_curve(event=None):
    '''
    '''
    global myGroup
    display.EraseAll()
    P = gp_Pnt(1,2,3)
    distance, radius = 5, 5
    
    C = Geom_Circle(gp_XOY(),radius)
    PPC = GeomAPI_ProjectPointOnCurve(P,C.GetHandle())                     
    N = PPC.NearestPoint()
    NbResults = PPC.NbPoints()
    
    edg  = make_edge(C.GetHandle())
    display.DisplayShape(edg)
    
    if NbResults > 0:
        for i in range(1,NbResults+1):
            Q = PPC.Point(i)
            distance = PPC.Distance(i)
            # do something with Q or distance here
                
    make_text("P", P, TEXT_HEIGHT)
    display.DisplayShape(make_vertex(P) , update=True)
    
    pstring = "N : at Distance : " + PPC.LowerDistance()       
    make_text(pstring, N, TEXT_HEIGHT)
    if NbResults > 0:
        for i in range(1,NbResults+1):
            Q = PPC.Point(i)
            distance = PPC.Distance(i)
            pstring = "Q" + i + ": at Distance :" + PPC.Distance(i)
            make_text(pstring, Q, TEXT_HEIGHT)
            display.DisplayShape(make_vertex(Q))
    print ("completed project_point_on_curve, moving on...")

def point_from_projections(event=None):
    r"""Point from projections"""
    display.EraseAll()
    P = gp_Pnt(7, 8, 9)
    radius = 5
    SP = Geom_SphericalSurface(gp_Ax3(gp_XOY()), radius)
    
    display.DisplayShape(make_face(SP.GetHandle()))

    PPS = GeomAPI_ProjectPointOnSurf(P, SP.GetHandle())
    N = PPS.NearestPoint()
    NbResults = PPS.NbPoints()
    if NbResults > 0:
        for i in range(1,NbResults+1):
            Q = PPS.Point(i)
            distance = PPS.Distance(i)

    display.DisplayShape(make_vertex(P), update=True)
    make_text("P", P, TEXT_HEIGHT)
    
    pstring = "N  : at Distance : " + repr(PPS.LowerDistance())
    
    display.DisplayShape(make_vertex(N))
    make_text(pstring, N, TEXT_HEIGHT)

    if NbResults > 0:
        for i in range(1,NbResults+1):
            Q = PPS.Point(i)
            distance = PPS.Distance(i)
            pstring = "Q" + i + ": at Distance :" + PPS.Distance(i)
            make_text(pstring, Q, TEXT_HEIGHT)
        
    print ("completed point_from_curve, moving on...")


def points_from_intersection(event=None):
    r"""Points from intersection"""
    display.EraseAll()
    PL = gp_Pln(gp_Ax3(gp_XOY()))
    MinorRadius, MajorRadius = 5, 8    
    
    EL = gp_Elips( gp_YOZ(), MajorRadius, MinorRadius)

    print (OCC.Precision.precision_Angular())
    ICQ = IntAna_IntConicQuad(EL, PL, OCC.Precision.precision_Angular(), OCC.Precision.precision_Confusion())
     
    if ICQ.IsDone():
        NbResults = ICQ.NbPoints()
        if NbResults > 0:
            for i in range(1, NbResults + 1):
                P = ICQ.Point(i)
   
    aPlane = GC_MakePlane(PL).Value()
    aSurface = Geom_RectangularTrimmedSurface(aPlane, - 8., 8., - 12., 12., True, True)
    display.DisplayShape(make_face(aSurface.GetHandle()), update=True)
        
    anEllips = GC_MakeEllipse(EL).Value()
    display.DisplayShape(make_edge(anEllips))
     
    if ICQ.IsDone():
        NbResults = ICQ.NbPoints()
        if NbResults > 0:
            for i in range(1, NbResults + 1):
                P = ICQ.Point(i)
                pstring = "P" + repr(i)
                display.DisplayShape(make_vertex(P))
                make_text(pstring, P, TEXT_HEIGHT)


def parabola(event=None):
    r"""Parabola"""
    display.EraseAll()
    # P is the vertex point                  
    # P and D give the axis of symmetry     
    # 6 is the focal length of the parabola 
    pnt2d = gp_Pnt2d(2, 3)
    direction2d = gp_Dir2d(4, 5)
    axis2_2d = gp_Ax22d(pnt2d, direction2d, True)
    parabola2d = gp_Parab2d(axis2_2d, 6)
    display.DisplayShape(make_vertex(pnt2d))
    make_text("P", pnt2d, TEXT_HEIGHT)

    a_parabola = GCE2d_MakeParabola(parabola2d)
    a_parabola_value = a_parabola.Value()
    
    a_trimmed_curve = Geom2d_TrimmedCurve(a_parabola_value, -100, 100, True)
    
    display.DisplayShape(make_edge2d(a_trimmed_curve.GetHandle()), update=True)
                   
    print (" The entity A of type gp_Ax22d is not displayable \n ")
    print (" The entity D of type gp_Dir2d is displayed as a vector \n    ( mean with a length != 1 ) \n ")


def axis(event=None):
    r"""Axis"""
    display.EraseAll()
    p1 = gp_Pnt(2, 3, 4)
    direction = gp_Dir(4, 5, 6)
    axis3 = gp_Ax3(p1, direction)
    IsDirectA = axis3.Direct()
           
    AXDirection = axis3.XDirection()
    AYDirection = axis3.YDirection()
           
    p2 = gp_Pnt(5, 3, 4)
    axis3_2 = gp_Ax3(p2, direction)
    axis3_2.YReverse()
    # axis3 is now left handed                
    IsDirectA2 = axis3_2.Direct()
                                       
    A2XDirection = axis3_2.XDirection()
    A2YDirection = axis3_2.YDirection()
     
    display.DisplayShape(make_vertex(p1), update=True)
    make_text("P1", p1, TEXT_HEIGHT)

    display.DisplayShape(make_vertex(p2), update=True)
    make_text("P2", p2, TEXT_HEIGHT)


def bspline(event=None):
    r"""BSpline"""
    display.EraseAll()
    array = list()
    array.append(gp_Pnt2d(0, 0))
    array.append(gp_Pnt2d(1, 2))
    array.append(gp_Pnt2d(2, 3))
    array.append(gp_Pnt2d(4, 3))
    array.append(gp_Pnt2d(5, 5))
    
    pt2d_list = point2d_list_to_TColgp_Array1OfPnt2d(array)
    spline_curve_1 = Geom2dAPI_PointsToBSpline(pt2d_list).Curve()
    
    harray = TColgp_HArray1OfPnt2d(1, 5)
    harray.SetValue(1, gp_Pnt2d(7 + 0, 0))
    harray.SetValue(2, gp_Pnt2d(7 + 1, 2))
    harray.SetValue(3, gp_Pnt2d(7 + 2, 3))
    harray.SetValue(4, gp_Pnt2d(7 + 4, 3))
    harray.SetValue(5, gp_Pnt2d(7 + 5, 5))
    
    an_interpolation = Geom2dAPI_Interpolate(harray.GetHandle(), False, 0.01)
    an_interpolation.Perform()
    spline_curve_2 = an_interpolation.Curve()
    
    harray2 = TColgp_HArray1OfPnt2d(1, 5)
    harray2.SetValue(1, gp_Pnt2d(11, 0))
    harray2.SetValue(2, gp_Pnt2d(12, 2))
    harray2.SetValue(3, gp_Pnt2d(13, 3))
    harray2.SetValue(4, gp_Pnt2d(15, 3))
    harray2.SetValue(5, gp_Pnt2d(16, 5))
    
    an_interpolation_2 = Geom2dAPI_Interpolate(harray2.GetHandle(), True, 0.01)
    an_interpolation_2.Perform()
    spline_curve_3 = an_interpolation_2.Curve()
    
    i = 0
    for P in array:
        i += 1
        pstring = 'array' + repr(i)
        make_vertex(P)
        make_text(pstring, P, TEXT_HEIGHT)
     
    for j in range(harray.Lower(), harray.Upper()+1):
        i += 1
        pstring = 'harray' + repr(i)
        P = harray.Value(j)
        make_vertex(P)
        make_text(pstring, P, TEXT_HEIGHT)
       
    for j in range(harray2.Lower(), harray2.Upper()+1):
        i += 1
        pstring = 'harray2' + repr(i)
        P = harray2.Value(j)
        make_vertex(P)
        make_text(pstring, P, TEXT_HEIGHT)
    
    display.DisplayShape(make_edge2d(spline_curve_1), update=True)
    display.DisplayShape(make_edge2d(spline_curve_2), update=True)
    display.DisplayShape(make_edge2d(spline_curve_3), update=True)


def curves2d_from_curves(event=None):
    r"""Curves 2d from curves"""
    display.EraseAll()
    major, minor = 12, 4                                              
    axis = gp_OX2d()                                             
    
    ellipse = GCE2d_MakeEllipse(axis, major, minor)
    ellipse_value = ellipse.Value()
     
    trimmed_curve = Geom2d_TrimmedCurve(ellipse_value, -1, 2, True)
    bspline_curve = OCC.Geom2dConvert.geom2dconvert_CurveToBSplineCurve(trimmed_curve.GetHandle(),
                                                                        OCC.Convert.Convert_TgtThetaOver2)
    display.DisplayShape(make_edge2d(bspline_curve), update=True)


def curves2d_from_offset(event=None):
    r"""Curves 2d from offset"""
    display.EraseAll()
    array = list()
    array.append(gp_Pnt2d(-4, 0))
    array.append(gp_Pnt2d(-7, 2))
    array.append(gp_Pnt2d(-6, 3))
    array.append(gp_Pnt2d(-4, 3))
    array.append(gp_Pnt2d(-3, 5))
    
    xxx = point2d_list_to_TColgp_Array1OfPnt2d(array)
    
    spline_curve_1 = Geom2dAPI_PointsToBSpline(xxx).Curve()
     
    dist = 1
    offset_curve = Geom2d_OffsetCurve(spline_curve_1, dist)
    result = offset_curve.IsCN(2)
     
    dist2 = 1.5
    offset_curve_2 = Geom2d_OffsetCurve(spline_curve_1, dist2)
    result2 = offset_curve_2.IsCN(2)
              
    display.DisplayShape( make_edge2d(spline_curve_1), update=True )
    display.DisplayShape( make_edge2d(offset_curve.GetHandle()), update=True)
    display.DisplayShape( make_edge2d(offset_curve_2.GetHandle()), update=True)


def circles2d_from_curves(event=None):
    r"""Circles 2d from curves"""
    display.EraseAll()
    p1 = gp_Pnt2d(9, 6)
    p2 = gp_Pnt2d(10, 4)
    p3 = gp_Pnt2d(6, 7)
    circle_2d = gce_MakeCirc2d(p1, p2, p3).Value()

    gccent_outside = OCC._GccEnt.gccent_Outside(circle_2d)
    p4 = gp_Pnt2d(-2, 7)
    p5 = gp_Pnt2d(12, -3)

    lin2d_2tan = GccAna_Lin2d2Tan(p4, p5, OCC.Precision.precision_Confusion()).ThisSolution(1)
     
    QL = OCC._GccEnt.gccent_Unqualified(lin2d_2tan)
    radius = 2.
    TR = GccAna_Circ2d2TanRad(gccent_outside, QL, radius, OCC.Precision.precision_Confusion())
    
    if TR.IsDone():
        nb_solutions = TR.NbSolutions()
        for k in range(1, nb_solutions + 1):
            circ = TR.ThisSolution(k)
            # find the solution circle                             
            pnt1 = gp_Pnt2d()
            parsol, pararg = TR.Tangency1(k, pnt1)
            # find the first tangent point                                    
            pnt2 = gp_Pnt2d()
            parsol,pararg = TR.Tangency2(k, pnt2)
            # find the second tangent point                                    
     
    make_text("P1", p1, TEXT_HEIGHT)
    make_text("P2", p2, TEXT_HEIGHT)
    make_text("P3", p3, TEXT_HEIGHT)
    make_text("P4", p4, TEXT_HEIGHT)
    make_text("P5", p5, TEXT_HEIGHT)
    [display.DisplayShape(make_vertex(i)) for i in [p1, p2, p3, p4, p5]]
    
    display.DisplayShape(make_edge2d(circle_2d))

    a_line = GCE2d_MakeSegment(lin2d_2tan, -2, 20).Value()
    display.DisplayShape(make_edge2d(a_line))

    if TR.IsDone():
        nb_solutions = TR.NbSolutions()
        for k in range(1, nb_solutions + 1):
            circ = TR.ThisSolution(k)
            a_circle = Geom2d_Circle(circ)
            display.DisplayShape(make_edge2d(a_circle.GetHandle()), update=True)
            # find the solution circle ( index, outvalue, outvalue, gp_Pnt2d )
            pnt3 = gp_Pnt2d()                         
            parsol, pararg = TR.Tangency1(k, pnt3)
            # find the first tangent point                                    
            make_text("tangentpoint1", pnt3, TEXT_HEIGHT)
            pnt4 = gp_Pnt2d()                         
            parsol, pararg = TR.Tangency2(k, pnt4)
            make_text("tangentpoint2", pnt4, TEXT_HEIGHT)


def curves3d_from_points(event=None):
    r"""Curves 3D from points"""
    display.EraseAll()
    p1 = gp_Pnt(0, 0, 1)
    p2 = gp_Pnt(1, 2, 2)
    p3 = gp_Pnt(2, 3, 3)
    p4 = gp_Pnt(4, 3, 4)
    p5 = gp_Pnt(5, 5, 5)
     
    array = list()
    array.append(p1)
    array.append(p2)
    array.append(p3)
    array.append(p4)
    array.append(p5)
     
    spline_curve_1 = GeomAPI_PointsToBSpline(point_list_to_TColgp_Array1OfPnt(array)).Curve()
    display.DisplayShape(make_edge(spline_curve_1))
    
    for i in range(0, len(array)):
        P = array[i]
        pstring = "P" + repr(i+1)
        if i == 0:
            pstring += " (array)  "
        display.DisplayShape(make_vertex(P), update=True)
        make_text(pstring, P, TEXT_HEIGHT)


def surface_from_curves(event=None):
    r"""Surface from curves"""
    display.EraseAll()
    array = list()
    array.append(gp_Pnt(-4, 0, 2))
    array.append(gp_Pnt(-7, 2, 2))
    array.append(gp_Pnt(-6, 3, 1))
    array.append(gp_Pnt(-4, 3, -1))
    array.append(gp_Pnt(-3, 5, -2))

    pt_list1 = point_list_to_TColgp_Array1OfPnt(array) 
    spline_1 = GeomAPI_PointsToBSpline(pt_list1).Curve()
    spline_1_object = spline_1.GetObject()
    
    a2 = list()
    a2.append(gp_Pnt(-4, 0, 2))
    a2.append(gp_Pnt(-2, 2, 0))
    a2.append(gp_Pnt(2, 3, -1))
    a2.append(gp_Pnt(3, 7, -2))
    a2.append(gp_Pnt(4, 9, -1))
    pt_list2 = point_list_to_TColgp_Array1OfPnt(a2)
    spline_2 = GeomAPI_PointsToBSpline(pt_list2).Curve()
    spline_2_object = spline_2.GetObject()
    
    a_geom_fill_1 = GeomFill_BSplineCurves(spline_1, spline_2, GeomFill_StretchStyle)
    
    spline_3 = Handle_Geom_BSplineCurve_DownCast(spline_1_object.Translated(gp_Vec(10, 0, 0)))
    spline_4 = Handle_Geom_BSplineCurve_DownCast(spline_2_object.Translated(gp_Vec(10, 0, 0)))
    a_geom_fill_2 = GeomFill_BSplineCurves(spline_3, spline_4, GeomFill_CoonsStyle)
    
    spline_5 = Handle_Geom_BSplineCurve_DownCast(spline_1_object.Translated(gp_Vec(20, 0, 0)))
    spline_6 = Handle_Geom_BSplineCurve_DownCast(spline_2_object.Translated(gp_Vec(20, 0, 0)))
    a_geom_fill_3 = GeomFill_BSplineCurves(spline_5, spline_6, GeomFill_CurvedStyle)
    
    a_bspline_surface_1 = a_geom_fill_1.Surface()
    a_bspline_surface_2 = a_geom_fill_2.Surface()
    a_bspline_surface_3 = a_geom_fill_3.Surface()
    
    # annotate
    make_text("GeomFill_StretchStyle", spline_1_object.StartPoint(), TEXT_HEIGHT)
    make_text("GeomFill_CoonsStyle", spline_3.GetObject().StartPoint(), TEXT_HEIGHT)
    make_text("GeomFill_CurvedStyle", spline_5.GetObject().StartPoint(), TEXT_HEIGHT)

    display.DisplayShape(make_face(a_bspline_surface_1))
    display.DisplayShape(make_face(a_bspline_surface_2))
    display.DisplayShape(make_face(a_bspline_surface_3), update=True)


def pipes(event=None):
    r"""Pipes"""
    display.EraseAll()
    a1 = list()
    a1.append(gp_Pnt(-4, 0, 2))
    a1.append(gp_Pnt(-5, 1, 0))
    a1.append(gp_Pnt(-6, 2, -2))
    a1.append(gp_Pnt(-5, 4, -7))
    a1.append(gp_Pnt(-3, 5, -12))
     
    pt_list1 = point_list_to_TColgp_Array1OfPnt(a1)
    spline_1 = GeomAPI_PointsToBSpline(pt_list1).Curve()
    display.DisplayShape(make_edge(spline_1))
    
    a_pipe = GeomFill_Pipe(spline_1, 1)  # second parameter is radius
    a_pipe.Perform(0, False)
    a_surface = a_pipe.Surface()
    
    an_ellipse = GC_MakeEllipse(gp_XOY(), 2, 1).Value()
    a_pipe_2 = GeomFill_Pipe(spline_1, an_ellipse, GeomFill_IsConstantNormal)
    a_pipe_2.Perform(0, False)
    a_surface_2 = a_pipe_2.Surface()
    a_surface_2.GetObject().Translate(gp_Vec(5, 0, 0))
    display.DisplayShape(make_face(a_surface_2))
    
    make_segment_1 = GC_MakeSegment(gp_Pnt(1, 1, 1), gp_Pnt(2, 2, 2)).Value()
    make_segment_2 = GC_MakeSegment(gp_Pnt(1, 1, 0), gp_Pnt(3, 2, 1)).Value()
    display.DisplayShape(make_edge(make_segment_1))
    display.DisplayShape(make_edge(make_segment_2))
    a_pipe_3 = GeomFill_Pipe(spline_1, make_segment_1, make_segment_2)
    a_pipe_3.Perform(0, False)
    a_surface_3 = a_pipe_3.Surface()
    a_surface_3.GetObject().Translate(gp_Vec(10, 0, 0))
    display.DisplayShape(make_face(a_surface))
    display.DisplayShape(make_face(a_surface_3))

    for i, mode in enumerate([GeomFill_IsConstantNormal, GeomFill_IsCorrectedFrenet, GeomFill_IsDarboux,
                              GeomFill_IsFrenet, GeomFill_IsGuideAC, GeomFill_IsGuideACWithContact,
                              GeomFill_IsGuidePlan, GeomFill_IsGuidePlanWithContact, ]):
        an_ellipse = GC_MakeEllipse(gp_XOY(), 2, 1).Value()
        display.DisplayShape(make_edge(an_ellipse))
    
        try:
            a_pipe_2 = GeomFill_Pipe(spline_1, make_segment_1, make_segment_2, mode)
            a_pipe_2.Perform(0, False)
            a_surface_2 = a_pipe_2.Surface()
            a_surface_2.GetObject().Translate(gp_Vec(5, 5, 0))
            display.DisplayShape(make_face(a_surface_2), update=True)
        except RuntimeError:
            print 'failed with mode:', mode


def bezier_surfaces(event=None):
    r"""Bezier surfaces"""
    display.EraseAll()
    array1 = TColgp_Array2OfPnt(1, 3, 1, 3)
    array2 = TColgp_Array2OfPnt(1, 3, 1, 3)
    array3 = TColgp_Array2OfPnt(1, 3, 1, 3)
    array4 = TColgp_Array2OfPnt(1, 3, 1, 3)
    
    array1.SetValue(1, 1, gp_Pnt(1, 1, 1))
    array1.SetValue(1, 2, gp_Pnt(2, 1, 2))
    array1.SetValue(1, 3, gp_Pnt(3, 1, 1))
    array1.SetValue(2, 1, gp_Pnt(1, 2, 1))
    array1.SetValue(2, 2, gp_Pnt(2, 2, 2))
    array1.SetValue(2, 3, gp_Pnt(3, 2, 0))
    array1.SetValue(3, 1, gp_Pnt(1, 3, 2))
    array1.SetValue(3, 2, gp_Pnt(2, 3, 1))
    array1.SetValue(3, 3, gp_Pnt(3, 3, 0))
    
    array2.SetValue(1, 1, gp_Pnt(3, 1, 1))
    array2.SetValue(1, 2, gp_Pnt(4, 1, 1))
    array2.SetValue(1, 3, gp_Pnt(5, 1, 2))
    array2.SetValue(2, 1, gp_Pnt(3, 2, 0))
    array2.SetValue(2, 2, gp_Pnt(4, 2, 1))
    array2.SetValue(2, 3, gp_Pnt(5, 2, 2))
    array2.SetValue(3, 1, gp_Pnt(3, 3, 0))
    array2.SetValue(3, 2, gp_Pnt(4, 3, 0))
    array2.SetValue(3, 3, gp_Pnt(5, 3, 1))
    
    array3.SetValue(1, 1, gp_Pnt(1, 3, 2))
    array3.SetValue(1, 2, gp_Pnt(2, 3, 1))
    array3.SetValue(1, 3, gp_Pnt(3, 3, 0))
    array3.SetValue(2, 1, gp_Pnt(1, 4, 1))
    array3.SetValue(2, 2, gp_Pnt(2, 4, 0))
    array3.SetValue(2, 3, gp_Pnt(3, 4, 1))
    array3.SetValue(3, 1, gp_Pnt(1, 5, 1))
    array3.SetValue(3, 2, gp_Pnt(2, 5, 1))
    array3.SetValue(3, 3, gp_Pnt(3, 5, 2))
    
    array4.SetValue(1, 1, gp_Pnt(3, 3, 0))
    array4.SetValue(1, 2, gp_Pnt(4, 3, 0))
    array4.SetValue(1, 3, gp_Pnt(5, 3, 1))
    array4.SetValue(2, 1, gp_Pnt(3, 4, 1))
    array4.SetValue(2, 2, gp_Pnt(4, 4, 1))
    array4.SetValue(2, 3, gp_Pnt(5, 4, 1))
    array4.SetValue(3, 1, gp_Pnt(3, 5, 2))
    array4.SetValue(3, 2, gp_Pnt(4, 5, 2))
    array4.SetValue(3, 3, gp_Pnt(5, 5, 1))
    
    BZ1, BZ2, BZ3, BZ4 = Geom_BezierSurface(array1), Geom_BezierSurface(array2), Geom_BezierSurface(array3),\
                         Geom_BezierSurface(array4)
    
    bezierarray = TColGeom_Array2OfBezierSurface(1, 2, 1, 2)
    bezierarray.SetValue(1, 1, BZ1.GetHandle())
    bezierarray.SetValue(1, 2, BZ2.GetHandle())
    bezierarray.SetValue(2, 1, BZ3.GetHandle())
    bezierarray.SetValue(2, 2, BZ4.GetHandle())

    BB = GeomConvert_CompBezierSurfacesToBSplineSurface(bezierarray)
    if BB.IsDone():
        poles = BB.Poles().GetObject().Array2()
        uknots = BB.UKnots().GetObject().Array1()
        vknots = BB.VKnots().GetObject().Array1()
        umult = BB.UMultiplicities().GetObject().Array1()
        vmult = BB.VMultiplicities().GetObject().Array1()
        udeg = BB.UDegree()
        vdeg = BB.VDegree()
                                       
        BSPLSURF = Geom_BSplineSurface(poles, uknots, vknots, umult, vmult, udeg, vdeg, 0, 0)
        BSPLSURF.Translate(gp_Vec(0, 0, 2))
           
    display.DisplayShape(make_face(BSPLSURF.GetHandle()), update=True )


def surfaces_from_offsets(event=None):
    display.EraseAll()
    array1 = list()
    array1.append(gp_Pnt(-4, 5, 5))
    array1.append(gp_Pnt(-3, 6, 6))
    array1.append(gp_Pnt(-1, 7, 7))
    array1.append(gp_Pnt(0, 8, 8))
    array1.append(gp_Pnt(2, 9, 9))
    spline_curve_1 = GeomAPI_PointsToBSpline(point_list_to_TColgp_Array1OfPnt(array1)).Curve()
    
    array2 = list()
    array2.append(gp_Pnt(-4, 5, 2))
    array2.append(gp_Pnt(-3, 6, 3))
    array2.append(gp_Pnt(-1, 7, 4))
    array2.append(gp_Pnt(0, 8, 5))
    array2.append(gp_Pnt(2, 9, 6))
    spline_curve_2 = GeomAPI_PointsToBSpline(point_list_to_TColgp_Array1OfPnt(array2)).Curve()
    
    a_geom_fill_1 = GeomFill_BSplineCurves(spline_curve_1, spline_curve_2, GeomFill_StretchStyle)
    a_geom_surface = a_geom_fill_1.Surface()
    
    offset = 1                                                       
    geom_offset_surface = Geom_OffsetSurface(a_geom_surface, offset)
    offset = 2                                                                     
    geom_offset_surface_1 = Geom_OffsetSurface(a_geom_surface, offset)
    offset = 3                                                                     
    geom_offset_surface_2 = Geom_OffsetSurface(a_geom_surface, offset)
       
    display.DisplayShape(make_face(a_geom_surface))
    display.DisplayShape(make_face(geom_offset_surface.GetHandle()))
    display.DisplayShape(make_face(geom_offset_surface_1.GetHandle()))
    display.DisplayShape(make_face(geom_offset_surface_2.GetHandle()), update=True)


def surfaces_from_revolution(event=None):
    display.EraseAll()
    array = list()
    array.append(gp_Pnt(0, 0, 1))
           
    array.append(gp_Pnt(1, 2, 2))
    array.append(gp_Pnt(2, 3, 3))
    array.append(gp_Pnt(4, 3, 4))
    array.append(gp_Pnt(5, 5, 5))
    a_curve = GeomAPI_PointsToBSpline(point_list_to_TColgp_Array1OfPnt(array)).Curve()
    
    surface_of_revolution = Geom_SurfaceOfRevolution(a_curve, gp_OX())
    
    display.DisplayShape(make_edge(a_curve))
    display.DisplayShape(make_face(surface_of_revolution.GetHandle()), update=True)


def distances(event=None):
    display.EraseAll()
    array1 = list()
    array1.append(gp_Pnt(-5, 1, 2))
    array1.append(gp_Pnt(-5, 2, 2))
    array1.append(gp_Pnt(-5.3, 3, 1))
    array1.append(gp_Pnt(-5, 4, 1))
    array1.append(gp_Pnt(-5, 5, 2))
    spl1 = GeomAPI_PointsToBSpline(point_list_to_TColgp_Array1OfPnt(array1)).Curve()
                                                                                         
    array2 = list()
    array2.append(gp_Pnt(4, 1, 2))
    array2.append(gp_Pnt(4, 2, 2))
    array2.append(gp_Pnt(3.7, 3, 1))
    array2.append(gp_Pnt(4, 4, 1))
    array2.append(gp_Pnt(4, 5, 2))
    spl2 = GeomAPI_PointsToBSpline(point_list_to_TColgp_Array1OfPnt(array2)).Curve()
    
    a_geomfill_1 = GeomFill_BSplineCurves(spl1, spl2, GeomFill_StretchStyle)
    a_surf_1 = a_geomfill_1.Surface()
     
    array3 = [] 
    
    array3 = TColgp_Array2OfPnt(1, 5, 1, 5)
    array3.SetValue(1, 1, gp_Pnt(-4, -4, 5))
    array3.SetValue(1, 2, gp_Pnt(-4, -2, 5))
    array3.SetValue(1, 3, gp_Pnt(-4, 0, 4))
    array3.SetValue(1, 4, gp_Pnt(-4, 2, 5))
    array3.SetValue(1, 5, gp_Pnt(-4, 4, 5))
    
    array3.SetValue(2, 1, gp_Pnt(-2, -4, 4))
    array3.SetValue(2, 2, gp_Pnt(-2, -2, 4))
    array3.SetValue(2, 3, gp_Pnt(-2, 0, 4))
    array3.SetValue(2, 4, gp_Pnt(-2, 2, 4))
    array3.SetValue(2, 5, gp_Pnt(-2, 5, 4))
    
    array3.SetValue(3, 1, gp_Pnt(0, -4, 3.5))
    array3.SetValue(3, 2, gp_Pnt(0, -2, 3.5))
    array3.SetValue(3, 3, gp_Pnt(0, 0, 3.5))
    array3.SetValue(3, 4, gp_Pnt(0, 2, 3.5))
    array3.SetValue(3, 5, gp_Pnt(0, 5, 3.5))
    
    array3.SetValue(4, 1, gp_Pnt(2, -4, 4))
    array3.SetValue(4, 2, gp_Pnt(2, -2, 4))
    array3.SetValue(4, 3, gp_Pnt(2, 0, 3.5))
    array3.SetValue(4, 4, gp_Pnt(2, 2, 5))
    array3.SetValue(4, 5, gp_Pnt(2, 5, 4))
    
    array3.SetValue(5, 1, gp_Pnt(4, -4, 5))
    array3.SetValue(5, 2, gp_Pnt(4, -2, 5))
    array3.SetValue(5, 3, gp_Pnt(4, 0, 5))
    array3.SetValue(5, 4, gp_Pnt(4, 2, 6))
    array3.SetValue(5, 5, gp_Pnt(4, 5, 5))
    
    a_surf_2 = GeomAPI_PointsToBSplineSurface(array3).Surface()
    
    ess = GeomAPI_ExtremaSurfaceSurface(a_surf_1, a_surf_2)
    dist = ess.LowerDistance()
    a, b = gp_Pnt(), gp_Pnt()
    ess.NearestPoints(a, b)
    
    nb_extrema = ess.NbExtrema()
    for k in range(1, nb_extrema+1):
        p3, p4 = gp_Pnt(), gp_Pnt()
        ess.Points(k, p3, p4)
        a_curve = GC_MakeSegment(p3, p4).Value()
        display.DisplayShape(make_vertex(p3))
        display.DisplayShape(make_vertex(p4))
        display.DisplayShape(make_edge(a_curve))
        make_text("P3", p3, TEXT_HEIGHT)
        make_text("P4", p4, TEXT_HEIGHT)
    
    display.DisplayShape(make_face(a_surf_1))
    display.DisplayShape(make_face(a_surf_2))
    
    display.DisplayShape(make_edge(spl1))
    display.DisplayShape(make_edge(spl2))
     
    make_text("P1", a, TEXT_HEIGHT)
    make_text("P2", b, TEXT_HEIGHT)
    display.DisplayShape(make_vertex(a))
    display.DisplayShape(make_vertex(b), update=True)


def exit(event=None):
    sys.exit() 

if __name__ == '__main__':
        add_menu('geometry')
        for f in [point_from_curve,
                  project_point_on_curve,
                  point_from_projections,
                  points_from_intersection,
                  parabola,
                  axis,
                  bspline,
                  curves2d_from_curves,
                  curves2d_from_offset,
                  circles2d_from_curves,
                  curves3d_from_points,
                  surface_from_curves,
                  pipes,
                  bezier_surfaces,
                  surfaces_from_offsets,
                  surfaces_from_revolution,
                  distances,
                  exit
                  ]:
            add_function_to_menu('geometry', f)
        start_display()