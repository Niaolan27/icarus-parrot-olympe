import requests

resp = requests.get("http://10.0.14.33/api/v1/info/properties")

props = {p["key"]: p["value"] for p in resp.json()}
serial = props.get("ro.factory.serial")
model = props.get("ro.product.model")
name = props.get("persist.product.name")
version = props.get("ro.parrot.build.version")
variant = props.get("ro.parrot.build.variant")

print(f"Serial: {serial}")
print(f"Model: {model}")
print(f"Name: {name}")
print(f"Version: {version}")
print(f"Variant: {variant}")