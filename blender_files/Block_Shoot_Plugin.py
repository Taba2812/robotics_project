bl_info = {
    "name" : "Blocks Automation Script",
    "author" : "Dawwo-15",
    "version" : (0, 1),
    "blender" : (3, 3, 7),
    "location" : "View3D > Tool",
    "warning" : "WIP",
    "wiki_url" : "none",
    "category" : "Automate Process",
}

import bpy
import math
import mathutils
import os

class MyProperties(bpy.types.PropertyGroup):
    block_color: bpy.props.StringProperty (
        name="Color",
        description=":",
        default="Blue",
        maxlen=20,
    )
    
    block_name: bpy.props.StringProperty (
        name="Name",
        description=":",
        default="X1-Y1-Z1",
        maxlen=20,
    )
    
    distance_prop: bpy.props.FloatProperty (
        name="Distance",
        description=":",
        default=1,
        max=2,
        min=0,
    )

class EXAMPLE_OT_func_3(bpy.types.Operator):
    bl_idname = "example.func_3"
    bl_label = "Generate"
    
    def execute(self, context):
        distance = 0.5
        object_origin = context.object.location
        vertical_angles = [0,30,60]
        horizontal_angles = [-90,0,90]
        color = "BLUE"
        name = ""
        collection = bpy.data.collections.new("Cameras")
        context.scene.collection.children.link(collection)
        
        for ha in horizontal_angles:
            for va in vertical_angles:
                camera_location = object_origin + (distance * mathutils.Vector((math.cos(math.radians(abs(ha))) * math.cos(math.radians(va)) ,math.sin(math.radians(ha)) * math.cos(math.radians(va)),math.sin(math.radians(va)))))
                camera_rotation = mathutils.Euler((math.radians(90-va),0,math.radians(90+ha)))
                camera = bpy.ops.object.camera_add(location=camera_location, rotation=camera_rotation)
                context.object.name = "camera_" + str(va) + "_" + str(ha)
                collection.objects.link(context.object)
        
        self.report({'INFO'}, f"Position Recorded {self.bl_idname}")
        return {'FINISHED'}

class EXAMPLE_OT_func_4(bpy.types.Operator):
    bl_idname = "example.func_4"
    bl_label = "Cleanup"
    
    def execute(self, context):
        collection = bpy.data.collections.get('Cameras')
        for obj in collection.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
            
        bpy.data.collections.remove(collection)
        return {'FINISHED'}

class EXAMPLE_OT_func_5(bpy.types.Operator):
    bl_idname = "example.func_5"
    bl_label = "Render"
    
    def execute(self, context):
        distance = bpy.context.scene.MyProperties.distance_prop
        context.scene.render.resolution_x= (int)(500 * distance)
        context.scene.render.resolution_y= (int)(500 * distance)
        object_origin = context.object.location
        vertical_angles = [0,30,45, 60]
        horizontal_angles = [-90,-60,-45,-30,0,30,45,60,90]
        color = bpy.context.scene.MyProperties.block_color
        name = bpy.context.scene.MyProperties.block_name
        bpy.ops.object.camera_add(location=(0,0,0),rotation=(0,0,0))
        camera = bpy.data.objects["Camera"]
        context.object.data.clip_start = 0.01
        
        for ha in horizontal_angles:
            for va in vertical_angles:
                camera_location = object_origin + (distance * mathutils.Vector((math.cos(math.radians(abs(ha))) * math.cos(math.radians(va)) ,math.sin(math.radians(ha)) * math.cos(math.radians(va)),math.sin(math.radians(va)))))
                camera_rotation = mathutils.Euler((math.radians(90-va),0,math.radians(90+ha)))
                camera.location = camera_location
                camera.rotation_euler = camera_rotation
                
                filename = "Blocco_" + name + "_" + color + "_" + str(ha) + "-" + str(va) + ".png"
                bpy.context.scene.render.filepath = bpy.path.abspath("//renders/"+name+"/"+filename)
                bpy.ops.render.render(write_still = True)
        bpy.data.objects.remove(camera, do_unlink=True)
        return {'FINISHED'}

class EXAMPLE_OT_func_6(bpy.types.Operator):
    bl_idname = "example.func_6"
    bl_label = "Test Camera"
    
    def execute(self, context):
        object_origin = context.object.location
        bpy.ops.object.camera_add(location=(0,0,0),rotation=(0,0,0))
        camera = bpy.data.objects["Camera"]
        context.object.data.clip_start = 0.01
    
        camera_location = object_origin + mathutils.Vector((0.15,0,0))
        camera_rotation = mathutils.Euler((math.radians(90),0,math.radians(90)))
        camera.location = camera_location
        camera.rotation_euler = camera_rotation
                
        return {'FINISHED'}

class Automation (bpy.types.Panel):
    bl_label = "Test Panel"
    bl_idname = "PT_Automation"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'NewTab'
    
    def draw(self, context):
        layout = self.layout
        obj = context.object
        
        row = layout.row()
        row.label(text="Indentation", icon='CUBE')
        row = layout.row()
        row.operator("mesh.primitive_cube_add")
        row = layout.row()
        row.operator(EXAMPLE_OT_func_3.bl_idname)
        row = layout.row()
        row.operator(EXAMPLE_OT_func_4.bl_idname)
        row = layout.row()
        row.operator(EXAMPLE_OT_func_5.bl_idname)
        row = layout.row()
        row.operator(EXAMPLE_OT_func_6.bl_idname)
        layout.row()
        layout.prop(bpy.context.scene.MyProperties, "block_color")
        layout.row()
        layout.prop(bpy.context.scene.MyProperties, "block_name")
        layout.row()
        layout.prop(bpy.context.scene.MyProperties, "distance_prop")
        
        

classes = (EXAMPLE_OT_func_3, EXAMPLE_OT_func_4, EXAMPLE_OT_func_5, EXAMPLE_OT_func_6, Automation, MyProperties)        
        
def register():
    for cls in classes:
        bpy.utils.register_class(cls)
        
    bpy.types.Scene.MyProperties = bpy.props.PointerProperty(type=MyProperties)
    
def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.MyProperties
    
if __name__=="__main__":
    register()