import yaml
parts_dict={'parts':{'agvs':{'agv4':{'tray_id':0,"parts":[{'type':'pump','color':'green','quadrant':'1','rotation':'0'}]}}}}
parts_dict["parts"]["agvs"]["agv4"]["parts"].append({'type':'sensor','color':'green','quadrant':'3','rotation':'pi'})
print("The python dictionary is:")
print(parts_dict)
file=open("testing.yaml","w")
yaml.dump(parts_dict,file, sort_keys=False)
file.close()
print("YAML file saved.")