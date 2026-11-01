"""AIFR V14 CORNER HANDWASH + CASH COUNTER — source: user's upright hand-drawn L-shaped cafeteria sketch.
Run in Blender Scripting > Run Script. ALL dimensions are provisional metres.
Overwrites current scene; save a copy first. Retains 18 tables T03-T05 and T08-T22.
Coordinates: X right, Y upwards in top view, Z height.
Main entrance: notch/top edge of left arm. Side exits: opposite/right exterior wall.
"""
import bpy, math, random, json
from mathutils import Vector

# Reset old incorrect models (save backup first)
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
random.seed(23)

# Editable dimensional assumptions — floor is the union of two rectangles:
WIDTH=19.0; ARM_DEPTH=7.5; WING_START=9.0; TOTAL_DEPTH=17.5
WALL_H=3.2; WT=0.16
# Lower arm: x 0-19, y 0-7.5. Upright wing: x 9-19, y 7.5-17.5.

def material(name, color, metallic=0.0, roughness=.55):
    mat=bpy.data.materials.new(name)
    mat.diffuse_color=(*color,1)
    mat.use_nodes=True
    p=mat.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value=(*color,1)
    p.inputs['Metallic'].default_value=metallic
    p.inputs['Roughness'].default_value=roughness
    return mat
WHITE=material('warm ivory walls',(.89,.88,.84)); FLOOR=material('grey floor',(.43,.46,.48)); BAND=material('dark floor stripes',(.18,.19,.20)); METAL=material('brushed stainless steel',(.61,.64,.67),.75,.25)
COUNTER=material('counter silver',(.53,.57,.60),.45); DARK=material('counter trim',(.13,.17,.21)); GLASS=material('glass aqua',(.45,.73,.80),.05,.15)
WOOD=material('table silver top',(.75,.77,.78),.65,.22); ROBOT=material('robot green',(.07,.48,.29)); TEXT=material('blue labels',(.07,.13,.32)); SINK=material('hand wash basin',(.77,.82,.85),.6)
CH=[material('chair_'+s,c) for s,c in [('red',(.76,.12,.16)),('peach',(.88,.57,.42)),('blue',(.35,.59,.77)),('black',(.15,.16,.19)),('mint',(.51,.71,.65)),('brown',(.43,.28,.25))]]

def cube(name, xy, size, mat, z=None):
    x,y=xy; sx,sy,sz=size
    bpy.ops.mesh.primitive_cube_add(size=1,location=(x,y,sz/2 if z is None else z))
    o=bpy.context.object;o.name=name;o.dimensions=size
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    o.data.materials.append(mat)
    return o

def cyl(name, xy, radius, depth, mat, z):
    bpy.ops.mesh.primitive_cylinder_add(vertices=16,radius=radius,depth=depth,location=(*xy,z))
    o=bpy.context.object;o.name=name;o.data.materials.append(mat)
    return o

def tag(label, x,y,z=0.025,size=.26):
    bpy.ops.object.text_add(location=(x,y,z))
    ob=bpy.context.object;ob.name='LABEL_'+label.replace(' ','_')
    ob.data.body=label;ob.data.size=size;ob.data.extrude=.002
    ob.data.materials.append(TEXT)
    return ob

def wallx(name,x1,x2,y):
    assert x2>x1
    cube(name,((x1+x2)/2,y),(x2-x1,WT,WALL_H),WHITE)
def wally(name,x,y1,y2):
    assert y2>y1
    cube(name,(x,(y1+y2)/2),(WT,y2-y1,WALL_H),WHITE)

# L floor: NO floor outside these rectangles.
cube('FLOOR_arm',(WIDTH/2,ARM_DEPTH/2),(WIDTH,ARM_DEPTH,.16),FLOOR,z=-.08)
cube('FLOOR_wing',((WING_START+WIDTH)/2,(ARM_DEPTH+TOTAL_DEPTH)/2),(WIDTH-WING_START,TOTAL_DEPTH-ARM_DEPTH,.16),FLOOR,z=-.08)
# floor bands, kept thin for visibility
for i,y in enumerate([2.15,5.25]):cube(f'tile_band_arm_{i}',(9.5,y),(18.6,.32,.008),BAND,z=.005)
for i,x in enumerate([11.35,15.65]):cube(f'tile_band_wing_{i}',(x,12.5),(.26,9.5,.008),BAND,z=.006)

# Outer L perimeter, gaps are ACTUAL openings, no hidden barrier across arm/wing junction:
wallx('WALL_service_tea_sandwich_juice',0,WIDTH,0)
wally('WALL_main_course_side',0,0,ARM_DEPTH)
# inner/recessed L top wall has MAIN ENTRY at x=3.1..5.1
wallx('WALL_notch_before_main_entry',0,3.1,ARM_DEPTH)
wallx('WALL_notch_after_main_entry',5.1,WING_START,ARM_DEPTH)
# New 1.80 m sliding entrance beside Table T11 (at x=10.9,y=9.0).
# The inner L-shaped side wall is split around y = 8.10 .. 9.90.
# THIS IS A REAL OPENING: no full-height wall or glass crossing the doorway.
SLIDE_Y0, SLIDE_Y1 = 8.10, 9.90
wally('WALL_inner_L_below_T11_sliding_door', WING_START, ARM_DEPTH, SLIDE_Y0)
wally('WALL_inner_L_above_T11_sliding_door', WING_START, SLIDE_Y1, TOTAL_DEPTH)
# Metal vertical jambs, thin threshold and upper door track.
SLIDE_FRAME=material('sliding door aluminium frame',(.22,.29,.34),.70,.28)
SLIDE_GLASS=material('sliding door pale glass',(.58,.83,.88),.04,.08)
p=SLIDE_GLASS.node_tree.nodes['Principled BSDF']
p.inputs['Base Color'].default_value=(.58,.83,.88,.23)
try: SLIDE_GLASS.surface_render_method='DITHERED'
except Exception: pass
for nm, yy in [('low',SLIDE_Y0),('high',SLIDE_Y1)]:
    cube('SLIDING_DOOR_T11_JAMB_'+nm,(WING_START, yy),(.13,.10,2.45),SLIDE_FRAME,z=1.225)
cube('SLIDING_DOOR_T11_TOP_TRACK',(WING_START,(SLIDE_Y0+SLIDE_Y1)/2),(.20,SLIDE_Y1-SLIDE_Y0,.10),SLIDE_FRAME,z=2.46)
cube('SLIDING_DOOR_T11_THRESHOLD',(WING_START,(SLIDE_Y0+SLIDE_Y1)/2),(.26,SLIDE_Y1-SLIDE_Y0,.022),METAL,z=.015)
# Glass panel is shown in the OPEN position, parked alongside adjacent solid wall.
# It is not drawn across the walk-through opening.
cube('SLIDING_DOOR_T11_OPEN_GLASS_PANEL',(WING_START-.14,10.72),(.035,1.45,2.18),SLIDE_GLASS,z=1.21)
cube('SLIDING_DOOR_T11_HANDLE',(WING_START-.10,10.10),(.11,.035,.30),METAL,z=1.05)
tag('SLIDING ENTRY BY T11',9.35,8.18,size=.17)
wallx('WALL_wing_far_end',WING_START,WIDTH,TOTAL_DEPTH)
# Opposite exterior wall: side gates at y=3.0..4.6 and 10.3..11.9
wally('WALL_right_below_side_gate_1',WIDTH,0,3.0)
wally('WALL_right_between_side_gates',WIDTH,4.6,10.3)
wally('WALL_right_above_side_gate_2',WIDTH,11.9,TOTAL_DEPTH)
# Threshold lines (not walls)
for name,x,y,along in [('MAIN_ENTRY',4.1,7.5,'X'),('SIDE_GATE_1',19,3.8,'Y'),('SIDE_GATE_2',19,11.1,'Y')]:
    if along=='X': cube('DOOR_THRESHOLD_'+name,(x,y),(2,.15,.01),DARK,z=.012)
    else:cube('DOOR_THRESHOLD_'+name,(x,y),(.15,1.6,.01),DARK,z=.012)
tag('MAIN ENTRY - L NOTCH',2.7,6.85,size=.23)
tag('SIDE EXIT 1',17.1,3.2,size=.23)
tag('SIDE EXIT 2',17.1,10.5,size=.23)

# Add visible OPEN sliding glass doors to all existing openings.
# Each panel is parked beside its opening; no floor-level glass blocks movement.
ACCENT_TEAL=material('accent teal',(.02,.56,.61),.10,.34)
ACCENT_CORAL=material('accent coral',(.96,.29,.32),.06,.48)
ACCENT_YELLOW=material('accent sunshine',(.99,.73,.15),.02,.45)
ACCENT_BLUE=material('accent cobalt',(.13,.33,.76),.08,.43)
ACCENT_GREEN=material('accent fresh green',(.15,.67,.46),.06,.45)

def sliding_horizontal(name, x0, x1, y, park='left'):
    mid=(x0+x1)/2; width=x1-x0
    cube(name+'_OVERHEAD_TRACK',(mid,y), (width+.18,.16,.12),SLIDE_FRAME,z=2.46)
    for key,xx in [('LEFT',x0),('RIGHT',x1)]:
        cube(name+'_JAMB_'+key,(xx,y),(.09,.12,2.40),SLIDE_FRAME,z=1.20)
    cube(name+'_LOW_THRESHOLD',(mid,y),(width,.15,.016),METAL,z=.012)
    panel_w=max(.70,width*.72)
    px=x0-panel_w/2-.10 if park=='left' else x1+panel_w/2+.10
    cube(name+'_OPEN_GLASS',(px,y+.06),(panel_w,.035,2.22),SLIDE_GLASS,z=1.20)
    cube(name+'_HANDLE',(px+(panel_w*.35 if park=='left' else -panel_w*.35),y+.105),(.025,.07,.26),METAL,z=1.10)
    cube(name+'_COLOUR_STRIP',(mid,y), (width,.065,.055),ACCENT_TEAL,z=2.56)

def sliding_vertical(name, x, y0, y1, park='below'):
    mid=(y0+y1)/2; width=y1-y0
    cube(name+'_OVERHEAD_TRACK',(x,mid),(.17,width+.18,.12),SLIDE_FRAME,z=2.46)
    for key,yy in [('LOW',y0),('HIGH',y1)]:
        cube(name+'_JAMB_'+key,(x,yy),(.12,.09,2.40),SLIDE_FRAME,z=1.20)
    cube(name+'_LOW_THRESHOLD',(x,mid),(.15,width,.016),METAL,z=.012)
    panel_w=max(.70,width*.72)
    py=y0-panel_w/2-.10 if park=='below' else y1+panel_w/2+.10
    cube(name+'_OPEN_GLASS',(x-.065,py),(.035,panel_w,2.22),SLIDE_GLASS,z=1.20)
    cube(name+'_HANDLE',(x-.105,py+(panel_w*.35 if park=='below' else -panel_w*.35)),(.07,.025,.26),METAL,z=1.1)

# Main entrance along top notch, and the two opposite side exits.
sliding_horizontal('MAIN_ENTRY_SLIDING',3.1,5.1,ARM_DEPTH,park='left')
sliding_vertical('SIDE_GATE_1_SLIDING',WIDTH,3.0,4.6,park='below')
sliding_vertical('SIDE_GATE_2_SLIDING',WIDTH,10.3,11.9,park='below')
# Existing T11 sliding doorway above is retained in its open position.


# Counters: hand wash, tea, sandwich, juice positioned IN ORDER along FRONT/LOWER WALL.
# Their front faces are toward +Y (inside room). No extra walls in front.
def food_counter(name,center,length):
    x,y=center
    cube('COUNTER_'+name,(x,y),(length,.85,1.05),COUNTER)
    cube('TOP_'+name,(x,y),(length+.04,.92,.06),METAL,z=1.075)
    # small back-mounted sneeze-glass strip, leaving service approach open at y>1.2
    cube('GLASS_'+name,(x,y-.22),(length-.10,.045,.40),GLASS,z=1.33)
    tag(name.replace('_',' '),x-length/2+.15,y+0.75,size=.23)
food_counter('TEA',(5.6,.65),2.3)
food_counter('SANDWICH',(8.65,.65),2.6)
# Juice counter now extends all the way to the end of the right-hand side wall.
# Tea and Sandwich remain fixed: Juice section occupies x=11.0..18.88.
# NOTE: right-hand side exit openings are on x=19 at y=3..4.6 and 10.3..11.9;
# this counter stays beside the front wall (y~0.65), so it blocks neither exit.
food_counter('JUICE',(14.94,.65),7.88)

# Segmented facade makes the long juice counter look like a unified service bay.
for k in range(6):
    cx=11.18+k*1.39
    cube(f'JUICE_SERVICE_PANEL_{k+1:02d}', (cx,1.103), (1.31,.027,.64), DARK, z=.51)
    cube(f'JUICE_PANEL_TRIM_{k+1:02d}', (cx,1.123), (1.28,.018,.028), METAL, z=.86)
# A small raised fascia/backboard frames the entire service line.
cube('JUICE_BACK_FASCIA',(14.94,.20),(7.88,.12,.56),WHITE,z=1.48)
tag('JUICE / COLD DRINKS / PICKUP',12.10,1.30,z=1.13,size=.24)

# Main course counter ALONG LEFT SIDE wall, near main entry (not across entrance)
cube('COUNTER_MAIN_COURSE',(0.67,5.0),(.92,2.8,1.05),COUNTER)
cube('TOP_MAIN_COURSE',(0.67,5.0),(1.00,2.88,.05),METAL,z=1.08)
tag('MAIN COURSE',1.35,5.7,size=.22)
# Corner handwash: sits at junction of LEFT service wall and FRONT counter wall.
# Kept separate from the Main Course counter, leaving the circulation area open.
HANDWASH_X, HANDWASH_Y = 0.82, 0.78
cube('HAND_WASH_CORNER_BASE',(HANDWASH_X,HANDWASH_Y),(.88,.85,.78),COUNTER,z=.39)
cube('HAND_WASH_CORNER_TOP',(HANDWASH_X,HANDWASH_Y),(.97,.94,.065),METAL,z=.81)
cyl('HAND_WASH_BASIN',(HANDWASH_X,HANDWASH_Y),.30,.105,SINK,z=.805)
cyl('HAND_WASH_DRAIN',(HANDWASH_X,HANDWASH_Y),.065,.009,DARK,z=.863)
cube('HAND_WASH_BACK_SPLASH',(HANDWASH_X,.22),(.92,.10,.60),WHITE,z=1.10)
cube('HAND_WASH_LEFT_SPLASH',(.23,HANDWASH_Y),(.10,.85,.60),WHITE,z=1.10)
tag('HAND WASH',.26,1.42,size=.19)

# Dedicated CASH COUNTER between Hand Wash and Tea.
# Tea starts at x=4.45. Cash counter spans x=2.00..3.65;
# small physical gaps remain between all service stations.
CASH_X, CASH_Y, CASH_LEN = 2.825, .65, 1.65
cube('COUNTER_CASH',(CASH_X,CASH_Y),(CASH_LEN,.85,1.05),COUNTER)
cube('CASH_COUNTER_METAL_TOP',(CASH_X,CASH_Y),(CASH_LEN+.05,.91,.065),METAL,z=1.08)
cube('CASH_COUNTER_FRONT_COLOUR',(CASH_X,1.095),(CASH_LEN-.10,.028,.45),ACCENT_BLUE,z=.67)
cube('CASH_COUNTER_RIM',(CASH_X,1.12),(CASH_LEN+.02,.035,.045),DARK,z=1.10)
cube('CASH_REGISTER_BODY',(CASH_X-.28,.50),(.40,.31,.20),DARK,z=1.23)
cube('CASH_REGISTER_SCREEN',(CASH_X-.28,.69),(.35,.035,.24),ACCENT_TEAL,z=1.34)
cube('CASH_PAYMENT_TERMINAL',(CASH_X+.40,.75),(.18,.24,.08),DARK,z=1.18)
tag('CASH / BILLING',CASH_X-.65,1.33,z=.025,size=.20)
# No connecting wall from Main Course to Hand Wash or from Hand Wash to Cash.

# Colourful decor inspired by mixed-colour cafeteria chairs (not structural walls).
# Slim coloured counter fronts are decorative and keep walkways clear.
for name, x, length, mat in [
    ('TEA',5.6,2.3,ACCENT_CORAL),
    ('SANDWICH',8.65,2.6,ACCENT_YELLOW),
    ('JUICE',14.94,7.88,ACCENT_TEAL)]:
    cube('COLOUR_COUNTER_FRONT_'+name,(x,1.10),(length-.1,.028,.30),mat,z=.64)
    cube('COLOUR_COUNTER_HEADER_'+name,(x,.20),(length-.1,.07,.12),mat,z=1.81)
cube('COLOUR_MAIN_COURSE_FACE',(1.15,5.0),(.035,2.45,.44),ACCENT_BLUE,z=.64)
cube('COLOUR_HANDWASH_FACE',(HANDWASH_X,1.245),(.76,.025,.22),ACCENT_GREEN,z=.48)
# Small decorative wall ribbons high above furniture; not traversability obstacles.
for i,(x,y,w,color) in enumerate([(2.0,.10,2.8,ACCENT_CORAL),(8.6,.10,3.0,ACCENT_YELLOW),(15.0,.10,5.6,ACCENT_TEAL)]):
    cube('COLOUR_WALL_RIBBON_'+str(i),(x,y),(w,.035,.12),color,z=2.70)
# Bright tile details flush with floor, kept away from entrance thresholds.
for i,(x,y,colour) in enumerate([(3.2,2.2,ACCENT_TEAL),(7.5,2.2,ACCENT_YELLOW),(12.0,2.2,ACCENT_CORAL),(16.2,5.2,ACCENT_BLUE)]):
    cube('COLOUR_FLOOR_TILE_'+str(i),(x,y),(.45,.45,.003),colour,z=.004)

# Photo based aesthetics: white square columns + transverse overhead beams.
columns=[(9.0,7.5),(13.5,7.5),(9.0,13.0),(13.5,13.0),(4.0,4.3),(9.0,4.3)]
for i,(x,y) in enumerate(columns,1):cube(f'PILLAR_{i:02d}',(x,y),(.38,.38,WALL_H),WHITE)
for i,y in enumerate([4.3,7.5,13.0]):
    if y==4.3:cube('CEILING_BEAM_'+str(i),(9.5,y),(18.8,.32,.42),WHITE,z=WALL_H-.06)
    else:cube('CEILING_BEAM_'+str(i),(14.0,y),(9.9,.32,.42),WHITE,z=WALL_H-.06)

# Four full chairs per table, with correct rotation of seat AND back via parent pivot.
def chair(name, cx,cy,heading,mat):
    root=bpy.data.objects.new(name,None);bpy.context.collection.objects.link(root)
    root.location=(cx,cy,0);root.rotation_euler[2]=heading
    def part(n,offset,sz,m):
        ob=cube(name+'_'+n,(cx+offset[0],cy+offset[1]),sz,m,z=offset[2]);ob.parent=root
        # parenting while retaining local coordinate placement correctly
        ob.matrix_parent_inverse=root.matrix_world.inverted()
        return ob
    # Instead use mesh local transforms parent inverse: build at global transformed position
    def localpart(n,offs,size,m):
        ob=cube(name+'_'+n,(0,0),size,m,z=0);ob.parent=root
        ob.location=offs
    localpart('SEAT',(0,0,.43),(.39,.39,.055),mat)
    localpart('BACK',(0,-.16,.69),(.39,.055,.52),mat)
    for a in [-.145,.145]:
        for b in [-.145,.145]:localpart('LEG'+str(a)+str(b),(a,b,.21),(.025,.025,.42),METAL)

def table(i,x,y):
    name=f'T{i:02d}'
    cube(name+'_TOP',(x,y),(1.02,.64,.055),WOOD,z=.755)
    cube(name+'_STEM',(x,y),(.065,.065,.70),METAL,z=.36)
    cube(name+'_BASE',(x,y),(.55,.21,.055),METAL,z=.04)
    chair(name+'_N',x,y+.65,0,CH[(i+0)%len(CH)])
    chair(name+'_S',x,y-.65,math.pi,CH[(i+1)%len(CH)])
    chair(name+'_E',x+.84,y,math.pi/2,CH[(i+2)%len(CH)])
    chair(name+'_W',x-.84,y,-math.pi/2,CH[(i+3)%len(CH)])
    tag(name,x-.20,y-.10,z=.79,size=.15)

# Preserve the exact screenshot's 22 original table positions and numbering.
# Remove ONLY old tables 01, 02, 06 and 07, each with all four chairs.
# Remaining tables: T03-T05 and T08-T22 = 18 tables, 72 chairs.
original_positions = [(x,y) for y in (3.15,5.8) for x in (3.5,6.5,10.4,13.6,16.5)]
original_positions += [(x,y) for y in (9.0,11.25,13.5,15.75) for x in (10.9,14.0,17.0)]
removed_old_table_numbers = {1, 2, 6, 7}
remaining_tables = [(old_num, x, y)
                    for old_num, (x, y) in enumerate(original_positions, 1)
                    if old_num not in removed_old_table_numbers]
assert len(remaining_tables) == 18, f"Expected 18 tables, got {len(remaining_tables)}"
for old_num, x, y in remaining_tables:
    assert ((0.95 < x < WIDTH-.95 and .95 < y < ARM_DEPTH-.75)
            or (WING_START+1 < x < WIDTH-1 and ARM_DEPTH+.85 < y < TOTAL_DEPTH-1)), (old_num,x,y)
    table(old_num, x, y)

# Robot placeholder near main entrance: no motion until navigation controller is added.
cube('ROBOT_BODY',(5.9,6.45),(.40,.36,.30),ROBOT,z=.20)
cube('ROBOT_FOOD_TRAY',(5.9,6.45),(.32,.30,.04),DARK,z=.37)
tag('ROBOT START',5.2,6.1,size=.19)

# Light + camera; no opaque ceiling, so floor remains visible from above.
world=bpy.context.scene.world
if world:
    world.use_nodes=True
    world.node_tree.nodes['Background'].inputs['Strength'].default_value=.8
for i,(x,y) in enumerate([(4,4),(12,4),(14,10),(14,15)]):
    bpy.ops.object.light_add(type='AREA',location=(x,y,5.4))
    lamp=bpy.context.object;lamp.name=f'LIGHT_{i}';lamp.data.energy=1800;lamp.data.shape='DISK';lamp.data.size=7
bpy.ops.object.camera_add(location=(9.5,-19,24))
cam=bpy.context.object
look=Vector((9.5,8,0))-cam.location
cam.rotation_euler=look.to_track_quat('-Z','Y').to_euler()
bpy.context.scene.camera=cam
try:bpy.context.scene.render.engine='BLENDER_EEVEE_NEXT'
except Exception:pass
bpy.context.scene.render.resolution_x=1440;bpy.context.scene.render.resolution_y=1080
bpy.context.scene.render.resolution_percentage=100
# Save accessible layout data in blend custom properties for later 2D occupancy grid.
bpy.context.scene['AIFR_layout']='L-shape: arm x0..19 y0..7.5; wing x9..19 y7.5..17.5; approximate metres'
bpy.context.scene['AIFR_tables']=json.dumps({f'T{num:02d}':[x,y] for num,x,y in remaining_tables})
bpy.context.scene['AIFR_removed_old_tables']='V7 old numbers 01, 02, 06, 07'
bpy.context.scene['AIFR_doors']=json.dumps({'main_entry':[4.1,7.5],'side_exit_1':[19,3.8],'side_exit_2':[19,11.1],'sliding_entry_t11':[9.0,9.0]})
print('AIFR V14 created:', len(remaining_tables), 'tables,', 4*len(remaining_tables), 'chairs. All four openings use open sliding doors; corner handwash and cash counter added; no connecting service wall.')
