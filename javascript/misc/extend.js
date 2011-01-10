function extend(a, b) {
    for(var c in b) {
        a[c] = b[c];
    }
};

Object.prototype.isInstanceOf = function(_constructor) {
	obj = this;
	while(typeof obj === "object"){
		if(obj.constructor === _constructor)
			return true;
		obj = obj._super;
	}

	return false;
};