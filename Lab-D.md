# 700105 Simulation and Concurrency Lab Book

## Concurrency Lab D

*Date*

### Q1. *Exercise 1*

**Question:**

Compile and run the program.

Read through the code until you understand what is going on in the code.

The main program creates a single instance of class PBM, which runs in its own thread.

Add a second cube to the Scene class and a second instance of class PBM, for the new cube.

Is the animation smooth? If not, why not?

Now fix the issue.

**Solution:**

```c++
//Created two instances of PBM in Main
int WINAPI wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE, _In_ LPWSTR, _In_ int nShowCmd) {
	PBM pbm1;
	PBM pbm2;
	Scene scene(pbm1, pbm2);
	DxFramework app(scene);
	pbm1.start();
	pbm2.start();

	if (FAILED(app.initWindow(hInstance, nShowCmd)))
		return 0;

	if (FAILED(app.initDevice())) 
		return 0;

	MSG msg;
	msg.message = 0;
	while (WM_QUIT != msg.message) {
		if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else {
			app.render();
		}
	}

	pbm1.waitForTermination();
	pbm2.waitForTermination();
	return static_cast<int>(msg.wParam);
}
//Scene takes both PBMs
Scene(PBM& pbm, PBM& pbm2) : _pbm(pbm), _pbm2(pbm2) {}

// ----- First Cube -----
	const auto height1 = _pbm.height();
	XMMATRIX world1 = XMMatrixTranslation(-3, height1 - 10, 10);

	ConstantBuffer cb1{
		XMMatrixTranspose(world1),
		XMMatrixTranspose(_View),
		XMMatrixTranspose(_Projection)
	};

	pImmediateContext->UpdateSubresource(_pConstantBuffer, 0, nullptr, &cb1, 0, 0);
	pImmediateContext->DrawIndexed(36, 0, 0);

	// ----- Second Cube -----
	const auto height2 = _pbm2.height();
	XMMATRIX world2 = XMMatrixTranslation(3, height2 - 10, 10);

	ConstantBuffer cb2{
		XMMatrixTranspose(world2),
		XMMatrixTranspose(_View),
		XMMatrixTranspose(_Projection)
	};

//PBM now handles deltaTime
float PBM::deltaTime() {
	const auto time = std::chrono::high_resolution_clock::now();
	const auto deltaTime = (time - _prevTime).count() * 1e-9;
	_prevTime = time;

#ifdef _DEBUG
	std::stringstream sout;
	sout << "T= " << deltaTime << "\n" << std::ends;
	auto str = sout.str();
	auto wstr = std::wstring(str.begin(), str.end());
	OutputDebugString(wstr.c_str());
#endif
	
	return static_cast<float>(deltaTime);
}

void PBM::update() {
	auto deltaT = deltaTime();
}
```

**Reflection:**
The cause of the jittering was a the static delta time method which has a shared prevTime value. There was a race condition as one thread would read and change the value and the second thread would overwrite the value with old data causing jitters.
I decided to move delta time method into the PBM so each thread now has its own instance to do calculations on so there is no race condition.

### Q2. *Exercise 2*

**Question:**

Run the debugger. Once the program has started pause it. Select from the menu Debug | Windows | Threads. You will now be able to see both of the threads in the Thread window. Right click on one of the threads and select Freeze. Continue the debugger. That thread has now been paused. Pause the Debugger once more and then Thaw the thread. Continue the Debugger.

What causes the balls to bounce higher?

**Solution:**

```c++
N/A
```

**Reflection:**

When the thread was frozen then thawed was the cube shot off the screen. This is beacause when it was paused it stops doing calcuations however the program time keeps running so when its unpaused there is a huge difference between time and prevTime causing the object to shoot off the screen.
I also saw that sometimes the freezing of a thread could cause an impact on the value of the height of the cube that wasnt frozen I assume this is because the freezing of a thread can cause some blocking in main which slows down or speads up delta time calculations increasing or decreasing how much the cube moves.

### Q3. *Exercise 3*

**Question:**

Move the flatc compiler into the project folder

Open a console window in the project folder and execute ./flatc --cpp mydata.fbs to compiler the fbs file. This should create a mydata_generated.h.

Add the include to your source code.

Compile and run your new project. You should get a value of 56 bytes.

Add code to extract the data from the buffer.

**Solution:**

```c++
#include <iostream>
#include "flatbuffers/flatbuffers.h"
#include "mydata_generated.h"	

int main()
{
	// Create a FlatBufferBuilder
	flatbuffers::FlatBufferBuilder builder;

	// Create some data to serialize
	const auto pos = Lab700105::Vec3{ 1,2,3 };
	const auto col = Lab700105::RGBA{ 1.0, 0.5, 0.5, 1.0 };
	const auto node = Lab700105::CreateNode(builder, 10, &pos, &col);

	// Finish the buffer with the root object
	builder.Finish(node);

	// Get the pointer to the serialized data
	uint8_t* buf = builder.GetBufferPointer();
	const auto size = builder.GetSize();

	// Print the serialized data size
	std::cout << "Serialized data size: " << size << " bytes\n";

	// Extract data from the buffer
	auto extractedNode = Lab700105::GetNode(buf);

	// Extract the id
	uint64_t id = extractedNode->id();
	std::cout << "ID: " << id << std::endl;

	// Extract position (Vec3)
	auto position = extractedNode->position();
	if (position) {
		float x = position->x();
		float y = position->y();
		float z = position->z();
		std::cout << "Position: (" << x << ", " << y << ", " << z << ")" << std::endl;
	}

	// Extract colour (RGBA)
	auto colourData = extractedNode->colour();
	if (colourData) {
		float r = colourData->r();
		float g = colourData->g();
		float b = colourData->b();
		float a = colourData->a();
		std::cout << "Colour: RGBA(" << r << ", " << g << ", " << b << ", " << a << ")" << std::endl;
	}

	// Clean up and exit
	return 0;
}
```

**Sample output:**

Serialized data size: 56 bytes
ID: 10
Position: (1, 2, 3)
Colour: RGBA(1, 0.5, 0.5, 1)

**Reflection:**
First I put the commannd into the console which takes the schema file and compileds it into C++ code that handles all serialization/deserialization logic.
To create data you first make the structs containing the data you want to send. Then call create node and give an id and pointers to the data. Then use finish method which sets node as the root and the buffer is valid to send.
To read data you first

*Did you make any mistakes?*

*In what way has your knowledge improved?*

**Questions:**

*Is there anything you would like to ask?*

### Q4. *Question title*

**Question:**

*Question here*

**Solution:**

```c++
// code here
```

**Test data:**

*Delete if not required.*

**Sample output:**

*Delete if not required.*

**Reflection:**

*Reflect on what you have learnt from this exercise.*

*Did you make any mistakes?*

*In what way has your knowledge improved?*

**Questions:**

*Is there anything you would like to ask?*
