import static org.junit.jupiter.api.Assertions.*;

import com.alumiboti5590.util.filters.LinearInputFilter;
import org.junit.jupiter.api.Test;

public class LinearInputFilterTest {
  
  @Test
  void filtersPositiveNumbers() {
    double value = .2;
    double scaleFactor = 2;
    double expectedFilterValue = .4;

    LinearInputFilter filter = new LinearInputFilter(scaleFactor);
    assertEquals(expectedFilterValue, filter.get(value));
  }

  @Test
  void filtersNegativeNumbers() {
    double value = -.2;
    double scaleFactor = 2;
    double expectedFilterValue = -.4;

    LinearInputFilter filter = new LinearInputFilter(scaleFactor);
    assertEquals(expectedFilterValue, filter.get(value));
  }
}
