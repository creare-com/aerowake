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

/**
 * Register a callback lacking a this pointer.
 * That is, a static method, or a function that is not a class method, or a lambda.
 * @param callback function returning void and taking one argument
 */
template <typename ArgType>
void CallbackVector<ArgType>::registerStaticCallback(const CallbackVector<ArgType>::callbackFtnType &callback) {
	callbackVector.push_back(callback);
}
/**
 * Register a callback that is a non-static member function
 * @param callback member function returning void and taking one argument
 * @param callbackOwner the `this` pointer for the member function
 */
template <typename ArgType>
template <typename OwnerClass>
void CallbackVector<ArgType>::registerMemberCallback(void (OwnerClass::*callback)(ArgType), OwnerClass * callbackOwner) {
	if(callback != NULL) {
		registerStaticCallback(bind(callback, callbackOwner, placeholders::_1));
	}
}

/**
 * Call all callbacks registered to this callback vector.
 * @param arg the argument to pass to each callback
 */
template <typename ArgType>
void CallbackVector<ArgType>::fireCallbacks(ArgType arg) {
	for(auto callback : callbackVector) {
		callback(arg);
	}
}


int main() {
	// Declare a callback vector, whose callbacks take one integer argument
	CallbackVector<int> cbv;
	cbv.registerStaticCallback(TestClass::callMeStatic);
	TestClass testObj(1);
	cbv.registerMemberCallback<TestClass>(&TestClass::callMeMember, &testObj);
	cbv.fireCallbacks(2);
	
	return 0;
}