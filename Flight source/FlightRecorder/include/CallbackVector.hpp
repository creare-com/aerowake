/*
	CallbackVector.hpp
	
	Class for handling a vector of callbacks.
	
	
	Example:
	
	
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
		// Declare a callback vector, whose callbacks take one integer argument
		CallbackVector<int> cbv;
		cbv.registerCallback(TestClass::callMeStatic);
		TestClass testObj(1);
		cbv.registerCallback<TestClass>(&TestClass::callMeMember, &testObj);
		cbv.fireCallbacks(2);
		
		return 0;
	}
	
	
	
	2019-06-13	JDW	Created.
*/

#ifndef __CALLBACKVECTOR_HPP__
#define __CALLBACKVECTOR_HPP__

#include <vector>
#include <functional>

using namespace std;

template <typename ArgType>
class CallbackVector {
public:
	/**
	 * Constructor. 
	 */
	CallbackVector() { 
		
	}
	virtual ~CallbackVector() {
		
	}
	
	// Callbacks shall return void and take one argument
	typedef function<void(ArgType)> callbackFtnType;
	
	/**
	 * Register a callback without any `this` pointer.  (a static method,
	 * or a function that is not a class method, or a lambda.
	 * 
	 * Callbacks will be called on the same thread that calls fireCallbacks().
	 * Be careful about writing callbacks that acquire locks, or conduct 
	 * long-running operations such as IO, because when the callbacks
	 * are fired, they will occupy the thread that calls fireCallbacks().
	 * 
	 * @param callback function returning void and taking one argument
	 */
	void registerCallback(const callbackFtnType &callback);
	/**
	 * Register a callback that is a non-static member function
	 * 
	 * Callbacks will be called on the same thread that calls fireCallbacks().
	 * Be careful about writing callbacks that acquire locks, or conduct 
	 * long-running operations such as IO, because when the callbacks
	 * are fired, they will occupy the thread that calls fireCallbacks().
	 * 
	 * @tparam OwnerClass the type of param callbackOwner
	 * @param callback pointer to a member function returning void and taking one argument
	 * @param callbackOwner the `this` pointer for the member function
	 */
	template<typename OwnerClass>
	void registerCallback(void (OwnerClass::*callback)(ArgType), OwnerClass * callbackOwner);
	
	/**
	 * Call all callbacks registered to this callback vector.
	 * Remember this won't return until all callbacks have finished,
	 * so it could take a while.
	 * @param arg the argument to pass to each callback
	 */
	void fireCallbacks(ArgType arg);
	
private:
	vector<callbackFtnType> callbackVector;
};

#endif // __CALLBACKVECTOR_HPP__

