import klvdata # type:ignore
from klvdata.misb0601 import UASLocalMetadataSet, SensorLatitude, SensorLongitude # type:ignore

# Create metadata set
metadata_set = UASLocalMetadataSet(b'0x04f')

# Add sensor latitude and longitude
metadata_set.items.update([SensorLatitude(34.0522)])   # Example latitude
metadata_set.append(SensorLongitude(-118.2437)) # Example longitude

# Serialize to bytes
klv_bytes = bytes(metadata_set)

# Write to file
with open('klv_data_file.klv', 'wb') as f:
    f.write(klv_bytes)
