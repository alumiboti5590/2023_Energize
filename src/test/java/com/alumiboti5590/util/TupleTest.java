import static org.junit.jupiter.api.Assertions.*;

import com.alumiboti5590.util.Tuple;
import org.junit.jupiter.api.Test;


public class TupleTest {
  
  @Test
  void keepsValuesOrdered() {
    double firstNum = 55;
    double secondNum = 90;
    Tuple exampleTuple = new Tuple(firstNum, secondNum);
    assertEquals(firstNum, exampleTuple.first);
    assertEquals(secondNum, exampleTuple.second);
  }

  @Test
  void respectsEqualityWhenEqual() {
    double firstNum = 55;
    double secondNum = 90;
    Tuple exampleTuple = new Tuple(firstNum, secondNum);
    Tuple otherButEqualTuple = new Tuple(firstNum, secondNum);
    assertTrue(exampleTuple.equals(otherButEqualTuple), "expected two tuples to match with same values");
  }

  @Test
  void respectsEqualityWhenUnequal() {
    double firstNum = 55;
    double secondNum = 90;
    Tuple exampleTuple = new Tuple(firstNum, secondNum);
    Tuple otherButEqualTuple = new Tuple(secondNum, firstNum);
    assertFalse(exampleTuple.equals(otherButEqualTuple), "expected two tuples to not match with different values");
  }
}
