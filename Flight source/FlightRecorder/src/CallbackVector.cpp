/*
	CallbackVector.cpp
	
	Class for handling a vector of callbacks.
	
	2019-06-13	JDW	Created.
*/

#include <CallbackVector.hpp>
#include <iostream>

using namespace std;

class TestClass {
public:
	TestClass(int num) : num(num) { ; }
	void callMeMember(int secondNum) {
		cout << "TestClass object num " << num << " got: " << secondNum << endl;
	}
	static void callMeStatic (int secondNum) {
		cout << "TestClass statically got: " << secondNum << endl;
	}
private:
	int num;
};

int main() {
	
	CallbackVector<int> cbv;
	TestClass testObj(1);
	cbv.registerStaticCallback(&TestClass::callMeStatic);
	bind(&TestClass::callMeMember, &testObj, placeholders::_1);
	// cbv.registerMemberCallback(&TestClass::callMeMember, &testObj);
	cbv.fireCallbacks(2);
	
	return 0;
}