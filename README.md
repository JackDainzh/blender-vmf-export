## 20.05.26 Changes made:
1. Updated to support Blender 4.5.10.
2. Unified all scripts, excluding the material picker, since addon now handles UV maps automatically.
3. If the material base name is the same as in Hammer editor, e.g. brick/brickwall001 and the naming of the material in Blender is brick/brickwall001 AND the material exists in Hammer editor, Hammer will automatically pick it up and apply the UV coordinates from Blender.
4. Fixed the scaling issue when units from Blender (meters) would translate to Hammer with 1 to 1 ratio, making objects in Hammer very small.
5. Removed material picker script from the unified addon.
