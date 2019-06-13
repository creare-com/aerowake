/*
	CallbackVector.hpp
	
	Class for handling a vector of callbacks.
	
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
	typedef function<void(int)> callbackFtnType;
	
	/**
	 * Register a callback lacking a this pointer.
	 * That is, a static method, or a function that is not a class method, or a lambda.
	 * @param callback function returning void and taking one argument
	 */
	void registerStaticCallback(const callbackFtnType &callback);
	/**
	 * Register a callback that is a non-static member function
	 * @param callback member function returning void and taking one argument
	 * @param callbackOwner the `this` pointer for the member function
	 */
	void registerMemberCallback(const callbackFtnType &callback, void * callbackOwner);
	
	/**
	 * Call all callbacks registered to this callback vector.
	 * @param arg the argument to pass to each callback
	 */
	void fireCallbacks(ArgType arg);
	
private:
	
};

#endif // __CALLBACKVECTOR_HPP__

