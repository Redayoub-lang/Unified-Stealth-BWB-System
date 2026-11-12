from stealth_bwb_physics import StealthBWBPhysics

# إنشاء نسخة من طائرتك الموجودة في المشروع
my_drone = StealthBWBPhysics()

print("الموقع الابتدائي:", my_drone.pos)
print("الكتلة (كجم):", my_drone.mass)