bl_info = {
    "name": "VMF Toolkit Exporter",
    "author": "leaxcx (Updated for 4.5+)",
    "version": (1, 3, 0),
    "blender": (4, 5, 0),
    "location": "View3D > Sidebar > Level Editor",
    "description": "Export Blender geometry to Source VMF with perfect scale, right-side-up orientation, and stable face geometry.",
    "category": "Export",
}

import bpy
import os
import re
from mathutils import Vector, Matrix

from . import convex_shortcut

SCALE_FACTOR = 52.94117647

def apply_modifiers_to_obj(obj):
    bpy.context.view_layer.objects.active = obj
    modifiers_to_apply = [m.name for m in obj.modifiers]
    for mod_name in modifiers_to_apply:
        try:
            bpy.ops.object.modifier_apply(modifier=mod_name)
        except Exception:
            pass
        
def separate_loose_parts(obj):
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.separate(type='LOOSE')
    bpy.ops.object.mode_set(mode='OBJECT')

def calculate_uv_scale(obj, mesh, face):
    default_uv_scale = 0.25
    if obj.get("useMeshUV") == 1:
        if not mesh.uv_layers:
            return default_uv_scale
        uv_layer = mesh.uv_layers.active.data
        uvs = [uv_layer[loop_index].uv for loop_index in face.loop_indices]
        if not uvs:
            return default_uv_scale
        uv_width = max(uv.x for uv in uvs) - min(uv.x for uv in uvs)
        uv_height = max(uv.y for uv in uvs) - min(uv.y for uv in uvs)
        return (uv_width + uv_height) / 2 if (uv_width + uv_height) > 0 else default_uv_scale
    return default_uv_scale
    
def _get_material_image_size(obj, face, fallback=(512, 512)):
    """
    Try to find an Image Texture node size from the face's material.
    Returns (width, height). Falls back if not found.
    """
    try:
        if not obj.material_slots:
            return fallback
        if face.material_index >= len(obj.material_slots):
            return fallback

        mat = obj.material_slots[face.material_index].material
        if not mat or not mat.use_nodes or not mat.node_tree:
            return fallback

        for node in mat.node_tree.nodes:
            if node.type == 'TEX_IMAGE' and node.image:
                w, h = node.image.size
                if w and h:
                    return (int(w), int(h))
    except Exception:
        pass
    return fallback


def _pick_3_noncollinear_corners(mesh, face, obj_matrix_world, scale_factor):
    """
    Returns three corners as tuples: (P_world_scaled, UV, loop_index)
    Picks corners from loops so we get correct per-corner UVs.
    """
    uv_layer = mesh.uv_layers.active
    if not uv_layer:
        return None

    data = uv_layer.data
    corners = []
    used_verts = set()

    # Gather unique vertex corners first
    for li in face.loop_indices:
        vi = mesh.loops[li].vertex_index
        if vi in used_verts:
            continue
        used_verts.add(vi)

        p = obj_matrix_world @ mesh.vertices[vi].co
        p = Vector((p.x * scale_factor, p.y * scale_factor, p.z * scale_factor))
        uv = Vector((data[li].uv.x, data[li].uv.y))
        corners.append((p, uv, li))

        if len(corners) >= 3:
            break

    if len(corners) < 3:
        return None

    # Ensure non-collinear in 3D (area > epsilon)
    p1, _, _ = corners[0]
    for i in range(1, len(corners) - 0):
        for j in range(i + 1, len(corners)):
            p2 = corners[i][0]
            p3 = corners[j][0]
            area = (p2 - p1).cross(p3 - p1).length
            if area > 1e-6:
                return (corners[0], corners[i], corners[j])

    # Fallback: just first 3 (may fail solve later)
    return (corners[0], corners[1], corners[2])


def compute_vmf_texture_axes_for_face(obj, mesh, face, scale_factor, tex_size_fallback=(512, 512)):
    """
    Computes VMF uaxis/vaxis for a single Blender polygon face.

    Returns:
      (uaxis_vec3, u_shift, vaxis_vec3, v_shift, vmf_scale)
    where:
      uaxis string should be: f'"uaxis" "[{Ux} {Uy} {Uz} {u_shift}] {vmf_scale}"'
      vaxis string should be: f'"vaxis" "[{Vx} {Vy} {Vz} {v_shift}] {vmf_scale}"'

    Implementation details:
    - Builds a face-local 2D basis (e1,e2) on the plane.
    - Solves affine mapping: U = a*s + b*t + c, V = d*s + e*t + f
    - Converts to world-space planar axes.
    - Converts Blender UV (0..1) to texels using material image size if possible.
    """
    if not mesh.uv_layers:
        return None

    picked = _pick_3_noncollinear_corners(mesh, face, obj.matrix_world, scale_factor)
    if not picked:
        return None

    (P1, UV1, _), (P2, UV2, _), (P3, UV3, _) = picked

    # Face basis
    e1 = (P2 - P1)
    if e1.length < 1e-9:
        return None
    e1.normalize()

    # Use Blender's polygon normal but ensure it's in world space & scaled space direction
    # (normal direction unaffected by scale_factor since it's uniform)
    n = (obj.matrix_world.to_3x3() @ face.normal).normalized()
    # Build e2 perpendicular to e1 in face plane
    e2 = n.cross(e1)
    if e2.length < 1e-9:
        return None
    e2.normalize()

    # Convert UV to "texture pixels" so Hammer aligns like a real texture
    tex_w, tex_h = _get_material_image_size(obj, face, fallback=tex_size_fallback)
    U1 = UV1.x * tex_w
    V1 = (1.0 - UV1.y) * tex_h  # Blender UV origin differs; flip V for typical Hammer feel
    U2 = UV2.x * tex_w
    V2 = (1.0 - UV2.y) * tex_h
    U3 = UV3.x * tex_w
    V3 = (1.0 - UV3.y) * tex_h

    # Compute (s,t) coordinates for P2, P3 relative to P1
    # s(P1)=0,t(P1)=0 by construction
    s2 = (P2 - P1).dot(e1)
    t2 = (P2 - P1).dot(e2)
    s3 = (P3 - P1).dot(e1)
    t3 = (P3 - P1).dot(e2)

    # Solve affine parameters for U and V:
    # U = a*s + b*t + c ; with P1 => c = U1
    # U2-U1 = a*s2 + b*t2
    # U3-U1 = a*s3 + b*t3
    # Same for V.
    A = Matrix(((s2, t2),
                (s3, t3)))

    det = A.determinant()
    if abs(det) < 1e-12:
        # Degenerate in (s,t) space; can't represent as planar affine mapping.
        return None

    rhs_u = Vector((U2 - U1, U3 - U1))
    rhs_v = Vector((V2 - V1, V3 - V1))

    a, b = A.inverted() @ rhs_u
    d, e = A.inverted() @ rhs_v
    c = U1
    f = V1

    # Convert to world-space planar axis vectors
    Uvec = (a * e1) + (b * e2)
    Vvec = (d * e1) + (e * e2)

    # Shifts when expressing as: U(P) = dot(P, Uvec) + Ushift
    # Using expansion with anchor P1:
    Ushift = c - P1.dot(Uvec)
    Vshift = f - P1.dot(Vvec)

    # VMF's trailing "scale": keep at 1 and bake everything into axis vectors
    vmf_scale = 1.0

    return (Uvec, Ushift, Vvec, Vshift, vmf_scale)    

def rename_objects_in_collection(collection, start_id):
    idx = start_id
    for obj in collection.objects:
        if obj.type == 'MESH':
            obj.name = f"brush_{idx}"
            idx += 1

def parse_existing_vmf_entities(filepath):
    if not os.path.exists(filepath):
        return []
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        entities = []
        entity_blocks = re.findall(r'entity\s*\{[^}]*\}', content, re.DOTALL)
        for block in entity_blocks:
            if 'classname" "info_player_start"' not in block:
                entities.append(block)
        return entities
    except Exception:
        return []

class ExportVMFOperator(bpy.types.Operator):
    bl_idname = "export.vmf"
    bl_label = "Export VMF"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        settings = context.scene.vmf_export_settings
        target_path = bpy.path.abspath(settings.filepath)
        
        if not target_path or target_path == "//":
            self.report({'ERROR'}, "Please specify a valid file export path.")
            return {'CANCELLED'}
            
        write_vmf(context, target_path, rename_objects=settings.rename_objects)
        self.report({'INFO'}, f"Successfully exported to {target_path}")
        return {'FINISHED'}

def write_vmf(context, filepath, rename_objects):
    scene = context.scene
    collection_name = "brushes"
    collection = bpy.data.collections.get(collection_name)
    
    if not collection:
        print(f"Collection '{collection_name}' not found!")
        return

    if rename_objects:
        rename_objects_in_collection(collection, start_id=1)

    existing_entities = parse_existing_vmf_entities(filepath)
    id_counter = 1000 

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write('world\n{\n')
        f.write('\t"id" "1"\n')
        f.write('\t"skyname" "sky_day01_01"\n')
        f.write('\t"maxpropscreenwidth" "-1"\n')
        f.write('\t"detailvbsp" "detail.vbsp"\n')
        f.write('\t"detailmaterial" "detail/detailsprites"\n')
        f.write('\t"mapversion" "68"\n')
        f.write('\t"classname" "worldspawn"\n')

        for obj in scene.objects:
            if collection.name in [c.name for c in obj.users_collection]:
                if obj.type == 'MESH':
                    id_counter += 1
                    apply_modifiers_to_obj(obj)
                    
                    old_active = context.view_layer.objects.active
                    old_selected = list(context.selected_objects)
                    
                    bpy.ops.object.select_all(action='DESELECT')
                    obj.select_set(True)
                    context.view_layer.objects.active = obj
                    
                    separate_loose_parts(obj)
                    
                    f.write('\tsolid\n\t{\n')
                    f.write(f'\t\t"id" "{id_counter}"\n')
                    mesh = obj.data

                    for face in mesh.polygons:
                        if len(face.vertices) < 3:
                            continue
                        f.write('\t\tside\n\t\t{\n')
                        id_counter += 1
                        f.write(f'\t\t\t"id" "{id_counter}"\n')
                            
                        # Transform local vertex arrays into metric coordinate space
                        v1_world = obj.matrix_world @ mesh.vertices[face.vertices[0]].co
                        v2_world = obj.matrix_world @ mesh.vertices[face.vertices[1]].co
                        v3_world = obj.matrix_world @ mesh.vertices[face.vertices[2]].co

                        coords1 = (v1_world.x * SCALE_FACTOR, v1_world.y * SCALE_FACTOR, v1_world.z * SCALE_FACTOR)
                        coords2 = (v2_world.x * SCALE_FACTOR, v2_world.y * SCALE_FACTOR, v2_world.z * SCALE_FACTOR)
                        coords3 = (v3_world.x * SCALE_FACTOR, v3_world.y * SCALE_FACTOR, v3_world.z * SCALE_FACTOR)

                        # Reversing point sequence order (3 -> 2 -> 1) fixes face normal orientation for BSP compilers
                        f.write('\t\t\t"plane" "({:.6f} {:.6f} {:.6f}) ({:.6f} {:.6f} {:.6f}) ({:.6f} {:.6f} {:.6f})"\n'.format(*coords3, *coords2, *coords1))
                            
                        f.write('\t\t\tvertices_plus\n\t\t\t{\n')
                        f.write('\t\t\t\t"v" "({:.6f} {:.6f} {:.6f})"\n'.format(*coords3))
                        f.write('\t\t\t\t"v" "({:.6f} {:.6f} {:.6f})"\n'.format(*coords2))
                        f.write('\t\t\t\t"v" "({:.6f} {:.6f} {:.6f})"\n'.format(*coords1))
                        f.write('\t\t\t}\n')

                        material_name = "dev/dev_blendmeasure"
                        if obj.material_slots and face.material_index < len(obj.material_slots):
                            mat = obj.material_slots[face.material_index].material
                            if mat:
                                material_name = mat.name.upper()

                        f.write(f'\t\t\t"material" "{material_name}"\n')
                            
                        uv_scale = calculate_uv_scale(obj, mesh, face)
                        normal = face.normal
                        
                        axes = compute_vmf_texture_axes_for_face(obj, mesh, face, SCALE_FACTOR, tex_size_fallback=(512, 512))
                        if axes:
                            Uvec, Ushift, Vvec, Vshift, vmf_scale = axes
                            f.write('\t\t\t"uaxis" "[{:.6f} {:.6f} {:.6f} {:.6f}] {:.6f}"\n'.format(
                                Uvec.x, Uvec.y, Uvec.z, Ushift, vmf_scale
                            ))
                            f.write('\t\t\t"vaxis" "[{:.6f} {:.6f} {:.6f} {:.6f}] {:.6f}"\n'.format(
                                Vvec.x, Vvec.y, Vvec.z, Vshift, vmf_scale
                            ))
                        else:
                            # Fallback to your old heuristic if the UVs can't be represented as planar
                            uv_scale = calculate_uv_scale(obj, mesh, face)
                            normal = face.normal
                            if abs(normal.z) >= 0.9:
                                f.write('\t\t\t"uaxis" "[1 0 0 0] {:.6f}"\n'.format(uv_scale))
                                f.write('\t\t\t"vaxis" "[0 -1 0 0] {:.6f}"\n'.format(uv_scale))
                            elif abs(normal.y) >= 0.9:
                                f.write('\t\t\t"uaxis" "[1 0 0 0] {:.6f}"\n'.format(uv_scale))
                                f.write('\t\t\t"vaxis" "[0 0 -1 0] {:.6f}"\n'.format(uv_scale))
                            else:
                                f.write('\t\t\t"uaxis" "[0 1 0 0] {:.6f}"\n'.format(uv_scale))
                                f.write('\t\t\t"vaxis" "[0 0 -1 0] {:.6f}"\n'.format(uv_scale))
                            
                        f.write('\t\t\t"rotation" "0"\n')
                        f.write('\t\t\t"lightmapscale" "16"\n')
                        f.write('\t\t\t"smoothing_groups" "0"\n')
                        f.write('\t\t}\n')
                    f.write('\t}\n')
                    
                    bpy.ops.object.select_all(action='DESELECT')
                    for o in old_selected:
                        o.select_set(True)
                    context.view_layer.objects.active = old_active
                    
        f.write('}\n')

        # Export Entity Map Points
        for obj in scene.objects:
            if obj.type == 'EMPTY':
                id_counter += 1
                f.write('entity\n{\n')
                f.write(f'\t"id" "{id_counter}"\n')
                f.write('\t"origin" "{:.6f} {:.6f} {:.6f}"\n'.format(obj.location.x * SCALE_FACTOR, obj.location.y * SCALE_FACTOR, obj.location.z * SCALE_FACTOR)) 
                f.write('\t"angles" "{:.6f} {:.6f} {:.6f}"\n'.format(obj.rotation_euler.x, obj.rotation_euler.y, obj.rotation_euler.z)) 
                
                classname = obj.get("classname", "info_player_start" if obj.name.startswith("info_player_start") else "info_target")
                f.write(f'\t"classname" "{classname}"\n')
                
                for key in obj.keys():
                    if key not in ['_id', 'classname', 'cycles']:
                        f.write(f'\t"{key}" "{obj[key]}"\n')
                f.write('}\n')

        for entity_block in existing_entities:
            f.write(entity_block + '\n')

class VMFExportSettings(bpy.types.PropertyGroup):
    filepath: bpy.props.StringProperty(
        name="File Path",
        description="Path to save the VMF file",
        default="//map_output.vmf",
        subtype='FILE_PATH'
    )
    rename_objects: bpy.props.BoolProperty(
        name="Rename Objects to ID",
        description="Rename objects in the collection 'brushes' to their ID",
        default=True
    )

class VMFExportPanel(bpy.types.Panel):
    bl_label = "Export Scene to VMF"
    bl_idname = "PT_VMF_Export"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Level Editor'

    def draw(self, context):
        layout = self.layout
        settings = context.scene.vmf_export_settings
        layout.prop(settings, "filepath")
        layout.prop(settings, "rename_objects")
        layout.operator("export.vmf", text="Export VMF")

classes = (ExportVMFOperator, VMFExportSettings, VMFExportPanel)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.vmf_export_settings = bpy.props.PointerProperty(type=VMFExportSettings)
    convex_shortcut.register()

def unregister():
    convex_shortcut.unregister()
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.vmf_export_settings

if __name__ == "__main__":
    register()