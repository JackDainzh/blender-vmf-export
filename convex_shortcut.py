import bpy

class OBJECT_OT_make_mesh_convex(bpy.types.Operator):
    bl_idname = "object.make_mesh_convex"
    bl_label = "Make Mesh Convex"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        obj = context.object
        if obj and obj.type == 'MESH':
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.convex_hull()
            bpy.ops.object.mode_set(mode='OBJECT')
            self.report({'INFO'}, "Made Mesh Convex")
        else:
            self.report({'WARNING'}, "Selected object is not a mesh")
        return {'FINISHED'}

class VIEW3D_PT_make_mesh_convex_panel(bpy.types.Panel):
    bl_label = "Modeling Shortcuts"
    bl_idname = "VIEW3D_PT_make_mesh_convex_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Level Editor'

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.operator("object.make_mesh_convex")

classes = (OBJECT_OT_make_mesh_convex, VIEW3D_PT_make_mesh_convex_panel)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)