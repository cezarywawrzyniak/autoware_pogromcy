#!/usr/bin/env python
# coding: utf-8

# In[25]:


import json


# In[26]:


with open('trajectory.txt') as f:
    lines = f.readlines()
    


# In[27]:


X = []
Y = []
for i in lines:
#     print(i.split())
    X.append(float(i.split()[0]))
    Y.append(float(i.split()[1]))


# In[28]:


dic = dict()
dic = {"X": X, "Y": Y}


# In[29]:


out_file = open("trajectory.json", "w")
json.dump(dic, out_file, indent = 4, sort_keys = False)
out_file.close()


# In[ ]:




