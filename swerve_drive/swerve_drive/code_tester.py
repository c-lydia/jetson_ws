class Test:
    def __init__(self, x):
        self.x = x

msgs = []

for i in range(6):
    a = Test(i)
    msgs.append(a)

for i in msgs:
    print(i.x)